//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <unordered_set>
#include <cmath>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Eigenvalues>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.source_submap_idx_range_ = yaml.GetValue<int>("loop_closing", "source_submap_idx_range");
        options_.source_submap_stride_ = yaml.GetValue<int>("loop_closing", "source_submap_stride");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
        options_.save_loop_clouds_ = yaml.GetValue<bool>("loop_closing", "save_loop_clouds");
        options_.remove_ground_in_lc_ = yaml.GetValue<bool>("loop_closing", "remove_ground_in_lc");
        options_.max_submap_aspect_ = yaml.GetValue<double>("loop_closing", "max_submap_aspect");
        options_.max_centroid_dist_ = yaml.GetValue<double>("loop_closing", "max_centroid_dist");
        options_.min_coverage_rate_ = yaml.GetValue<double>("loop_closing", "min_coverage_rate");
        options_.coverage_cell_size_ = yaml.GetValue<double>("loop_closing", "coverage_cell_size");
        options_.suppress_radius_ = yaml.GetValue<double>("loop_closing", "suppress_radius");
        options_.suppress_time_ = yaml.GetValue<double>("loop_closing", "suppress_time");
        options_.suppress_kf_ = yaml.GetValue<int>("loop_closing", "suppress_kf");
        options_.gicp_reg_voxel_ = yaml.GetValue<double>("loop_closing", "gicp_reg_voxel");
        options_.gicp_reg_max_iter_ = yaml.GetValue<int>("loop_closing", "gicp_reg_max_iter");
        options_.gicp_reg_corr_dist_ = yaml.GetValue<double>("loop_closing", "gicp_reg_corr_dist");
        options_.enable_wall_refine_ = yaml.GetValue<bool>("loop_closing", "enable_wall_refine");
        options_.wall_refine_thresh_ = yaml.GetValue<double>("loop_closing", "wall_refine_thresh");
        options_.wall_refine_max_move_ = yaml.GetValue<double>("loop_closing", "wall_refine_max_move");
        options_.wall_normal_z_thr_ = yaml.GetValue<double>("loop_closing", "wall_normal_z_thr");

        // 新增键（reg_mode / thinwall_*）：旧配置不含这些键时保持默认值（GetValue 缺键抛异常，容错处理）
        try {
            options_.reg_mode_ = yaml.GetValue<std::string>("loop_closing", "reg_mode");
        } catch (...) {
        }
        try {
            options_.thinwall_max_outer_ = yaml.GetValue<int>("loop_closing", "thinwall_max_outer");
        } catch (...) {
        }
        try {
            options_.thinwall_huber_delta_ = yaml.GetValue<double>("loop_closing", "thinwall_huber_delta");
        } catch (...) {
        }
        try {
            options_.thinwall_knn_ = yaml.GetValue<int>("loop_closing", "thinwall_knn");
        } catch (...) {
        }
        try {
            options_.thinwall_nn_radius_ = yaml.GetValue<double>("loop_closing", "thinwall_nn_radius");
        } catch (...) {
        }
        try {
            options_.thinwall_floor_z_ = yaml.GetValue<double>("loop_closing", "thinwall_floor_z");
        } catch (...) {
        }
        try {
            options_.thinwall_ceil_z_ = yaml.GetValue<double>("loop_closing", "thinwall_ceil_z");
        } catch (...) {
        }
        LOG(INFO) << "[lc] reg_mode = " << options_.reg_mode_
                  << (options_.reg_mode_ == "thinwall" ? " (challenger: 单遍紧核薄墙配准)"
                                                       : " (incumbent: GICP+墙面精修)");
    }

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // 检测回环候选
    DetectLoopCandidates();

    if (options_.verbose_) {
        // LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    auto& kfs_mapping = all_keyframes_;
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        // LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    for (auto kf : kfs_mapping) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            continue;
        }

        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            /// 在同一条轨迹中，如果间隔太近，就不考虑回环
            break;
        }

        Vec3d dt = kf->GetOptBodyPose().translation() - cur_kf_->GetOptBodyPose().translation();
        double t2d = dt.head<2>().norm();  // x-y distance
        double range_th = options_.max_range_;

        if (t2d < range_th) {
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            c.Tij_ = kf->GetLIOBodyPose().inverse() * cur_kf_->GetLIOBodyPose();

            candidates_.emplace_back(c);
            check_first = kf;
        }
    }


    // ===== 空间去重 =====
    // 同一区域在 空间半径 + 时间窗 + 帧数窗 三个窗口内只触发一次优化。
    // 慢速行驶时关键帧密集（2m 内可堆十几帧），若不去重，
    // 会互相成为候选并反复重建全图。
    if (!candidates_.empty() && options_.suppress_radius_ > 1e-6) {
        const Vec3d cur_pos = cur_kf_->GetOptBodyPose().translation();
        const double cur_stamp = cur_kf_->GetState().timestamp_;
        const int cur_id = cur_kf_->GetID();

        for (const auto& r : recent_loop_regions_) {
            const double dist = (cur_pos - r.center).head<2>().norm();
            const double dt = cur_stamp - r.stamp;
            const int did = cur_id - r.kf_id;

            if (dist < options_.suppress_radius_ && dt < options_.suppress_time_ && did < options_.suppress_kf_) {
                if (options_.verbose_) {
                    LOG(INFO) << "[lc] suppress loop at (" << cur_pos.x() << "," << cur_pos.y() << "): dist=" << dist
                              << "m dt=" << dt << "s did=" << did << ", " << candidates_.size() << " candi dropped";
                }
                candidates_.clear();
                break;
            }
        }
    }

    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;
    }


    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (lc.ndt_score_ > options_.ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    // 与源子图对称：单边 4 帧、步长 2 → 各取 5 帧（与 rk100 一致）
    const int target_submap_idx_range = 4;
    const int target_submap_stride = 2;
    const int source_submap_idx_range = std::max(0, options_.source_submap_idx_range_);
    const int source_submap_stride = std::max(1, options_.source_submap_stride_);
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    auto build_submap = [this](int given_id, int idx_range, int idx_stride, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        for (int idx = -idx_range; idx <= idx_range; idx += idx_stride) {
            int id = idx + given_id;
            if (id < 0 || id >= all_keyframes_.size()) {
                continue;
            }

            auto kf = all_keyframes_[id];
            CloudPtr cloud = kf->GetCloud();

            // 地面过滤可开关（实测：走廊场景地面点占比仅 4~5%，移除几乎无收益，
            // 但会减少重叠区域，反而削弱大朝向差情况下的配准可靠性）。
            // RemoveGround 内部是 cloud->swap(...)，会原地改写共享对象，
            // 因此必须深拷贝，否则会永久裁掉关键帧点云。
            if (options_.remove_ground_in_lc_ && cloud && !cloud->empty()) {
                CloudPtr cloud_copy(new PointCloudType(*cloud));
                RemoveGround(cloud_copy, 0.1);
                cloud = cloud_copy;
            }

            if (!cloud || cloud->empty()) {
                continue;
            }

            // 转到世界系下
            SE3 Twb = kf->GetOptLidarPose();

            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetOptLidarPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
        }
        return submap;
    };

    auto submap_kf1 = build_submap(kf1->GetID(), target_submap_idx_range, target_submap_stride, true);

    auto submap_kf2 = build_submap(kf2->GetID(), source_submap_idx_range, source_submap_stride, false);

    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }

    Mat4f Tw2 = kf2->GetOptLidarPose().matrix().cast<float>();
    if (!Tw2.allFinite()) {
        LOG(ERROR) << "invalid initial pose for candidate " << c.idx1_ << "-" << c.idx2_;
        c.ndt_score_ = 0;
        return;
    }

    // 保存匹配前的源点云和目标点云（都变换到世界坐标系下，方便对比差距）
    // CloudPtr src_in_world(new PointCloudType);
    // pcl::transformPointCloud(*submap_kf2, *src_in_world, Tw2);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_before_src.pcd", *src_in_world);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_before_tgt.pcd", *submap_kf1);

    // ===== 过滤 1：狭长通道 =====
    // 任一子图的 XY 长宽比过大时判为狭长通道：这种子图在室内自相似环境中
    // 可以沿轴向滑到任意一条外观相同的走廊上，配准结果不可信。
    if (options_.max_submap_aspect_ > 0.0) {
        auto aspectOf = [](const CloudPtr& cloud) -> double {
            if (!cloud || cloud->size() < 30) return 0.0;
            Vec3d mean = Vec3d::Zero();
            int n = 0;
            for (const auto& pt : cloud->points) {
                if (!std::isfinite(pt.x)) continue;
                mean += Vec3d(pt.x, pt.y, 0.0);
                ++n;
            }
            if (n < 30) return 0.0;
            mean /= n;

            Mat2d cov = Mat2d::Zero();
            for (const auto& pt : cloud->points) {
                if (!std::isfinite(pt.x)) continue;
                Vec2d d(pt.x - mean.x(), pt.y - mean.y());
                cov += d * d.transpose();
            }
            cov /= n;

            Eigen::SelfAdjointEigenSolver<Mat2d> es(cov);
            const Vec2d major = es.eigenvectors().col(1), minor = es.eigenvectors().col(0);
            double lo_maj = 1e9, hi_maj = -1e9, lo_min = 1e9, hi_min = -1e9;
            for (const auto& pt : cloud->points) {
                if (!std::isfinite(pt.x)) continue;
                Vec2d d(pt.x - mean.x(), pt.y - mean.y());
                const double a = d.dot(major), b = d.dot(minor);
                lo_maj = std::min(lo_maj, a); hi_maj = std::max(hi_maj, a);
                lo_min = std::min(lo_min, b); hi_min = std::max(hi_min, b);
            }
            const double L = hi_maj - lo_maj, W = hi_min - lo_min;
            return L / std::max(W, 1e-6);
        };

        const double ar1 = aspectOf(submap_kf1);
        const double ar2 = aspectOf(submap_kf2);
        const double ar = std::max(ar1, ar2);
        if (ar > options_.max_submap_aspect_) {
            if (options_.verbose_) {
                LOG(WARNING) << "[lc] REJECT aspect " << c.idx1_ << "-" << c.idx2_ << ": ratio=" << ar
                             << " (tgt=" << ar1 << ", src=" << ar2 << ") > " << options_.max_submap_aspect_;
            }
            c.ndt_score_ = 0;
            return;
        }
    }

    // ===== 过滤 2：子图质心距 + XY 占据栅格覆盖度 =====
    // 先看两片子图的观测中心是否相距过远；超限时再判断覆盖度：
    // 只要一方的观测范围基本落在另一方范围内（包含关系），仍然是同一区域的两次观测，
    // 可以继续配准；若两片落在不同房间/通道，投影到地面的栅格几乎不重叠，直接拒绝。
    //
    // 用 XY 占据栅格而非逐点最近邻距离：同一面墙被两次扫描时点的采样位置不同，
    // 逐点距离天然偏大（0.1~0.2m），会把完全重合的两片子图误判为不重合。
    if (options_.max_centroid_dist_ > 0.0) {
        CloudPtr src_in_tgt(new PointCloudType);
        pcl::transformPointCloud(*submap_kf2, *src_in_tgt, Tw2);

        Vec2d c1 = Vec2d::Zero(), c2 = Vec2d::Zero();
        auto centroidXY = [](const CloudPtr& cloud, Vec2d& out) -> bool {
            Vec2d sum = Vec2d::Zero();
            int n = 0;
            for (const auto& pt : cloud->points) {
                if (!std::isfinite(pt.x)) continue;
                sum += Vec2d(pt.x, pt.y);
                ++n;
            }
            if (n < 10) return false;
            out = sum / n;
            return true;
        };

        if (centroidXY(submap_kf1, c1) && centroidXY(src_in_tgt, c2)) {
            c.centroid_dist_ = (c1 - c2).norm();
            if (options_.verbose_) {
                LOG(INFO) << "[lc] centroid " << c.idx1_ << "-" << c.idx2_ << ": " << c.centroid_dist_ << "m";
            }

            if (c.centroid_dist_ > options_.max_centroid_dist_) {
                const double cell = std::max(0.05, options_.coverage_cell_size_);
                auto occupancy = [cell](const CloudPtr& cloud) {
                    std::unordered_set<long long> cells;
                    cells.reserve(cloud->size());
                    for (const auto& pt : cloud->points) {
                        if (!std::isfinite(pt.x)) continue;
                        const long long ix = static_cast<long long>(std::floor(pt.x / cell));
                        const long long iy = static_cast<long long>(std::floor(pt.y / cell));
                        cells.insert((ix << 32) ^ (iy & 0xffffffffLL));
                    }
                    return cells;
                };
                auto coveredBy = [](const std::unordered_set<long long>& a,
                                    const std::unordered_set<long long>& b) -> double {
                    if (a.empty()) return 0.0;
                    size_t hit = 0;
                    for (long long k : a) {
                        if (b.count(k)) ++hit;
                    }
                    return 100.0 * hit / a.size();
                };

                const auto cells1 = occupancy(submap_kf1);
                const auto cells2 = occupancy(src_in_tgt);
                const double fwd = coveredBy(cells2, cells1);
                const double rev = coveredBy(cells1, cells2);
                c.coverage_rate_ = std::max(fwd, rev);

                if (options_.verbose_) {
                    LOG(INFO) << "[lc] coverage " << c.idx1_ << "-" << c.idx2_ << ": cell=" << cell
                              << " fwd=" << fwd << "% rev=" << rev << "% max=" << c.coverage_rate_ << "%";
                }

                if (c.coverage_rate_ < options_.min_coverage_rate_) {
                    if (options_.verbose_) {
                        LOG(WARNING) << "[lc] REJECT centroid " << c.idx1_ << "-" << c.idx2_ << ": dist="
                                     << c.centroid_dist_ << "m > " << options_.max_centroid_dist_
                                     << "m, coverage=" << c.coverage_rate_ << "% < " << options_.min_coverage_rate_
                                     << "%";
                    }
                    c.ndt_score_ = 0;
                    return;
                }
            }
        }
    }

    /// GICP 配准（与 rk100 回环配准算法一致）
    // LIO 初值精度足够，不需要 NDT 的粗级大范围搜索；
    // NDT 的粗分辨率在走廊/相似房间场景会被引向相邻的相似结构，
    // 表现为"几何贴合但房间错位"的假回环，GICP 从初值局部收敛不会跨房间跳变。
    CloudPtr output(new PointCloudType);

    const double voxel_size = std::max(0.1, options_.gicp_reg_voxel_);
    CloudPtr rough_map1 = VoxelGrid(submap_kf1, voxel_size);
    CloudPtr rough_map2 = VoxelGrid(submap_kf2, voxel_size);

    if (!rough_map1 || !rough_map2 || rough_map1->size() < 50 || rough_map2->size() < 50) {
        c.ndt_score_ = 0;
        return;
    }

    // NaN/Inf 清理：点云含非有限值时配准内部协方差会污染，导致不收敛
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*rough_map1, *rough_map1, indices);
        pcl::removeNaNFromPointCloud(*rough_map2, *rough_map2, indices);
        rough_map1->is_dense = false;
        rough_map2->is_dense = false;
    }
    if (rough_map1->size() < 50 || rough_map2->size() < 50) {
        c.ndt_score_ = 0;
        return;
    }

    // ===== 对准模式二选一（reg_mode）：gicp_wall（默认，GICP+墙中位精修）/ thinwall（NVMS 紧核挑战者） =====
    // 两种模式共用同一条验收尺子（全分辨率 submap_kf1 的墙中位残差 + move 上限）与
    // 同一评分口径（NN 均方距离 → 1/MSE → ndt_score_th_ 闸门），保证 A/B 可比。
    Quatd q = Quatd::Identity();
    Vec3d t = Vec3d::Zero();
    SE3 wall_refined_T;
    bool wall_refine_done = false;

    if (options_.reg_mode_ == "thinwall") {
        // ---- thinwall：不跑 GICP，从轨迹初值 T_init 出发做紧核薄墙配准 ----
        const auto tw_start = std::chrono::steady_clock::now();
        const Mat4d Ti = Tw2.cast<double>();
        Quatd qi(Ti.block<3, 3>(0, 0));
        qi.normalize();
        const SE3 T_init(qi, Ti.block<3, 1>(0, 3));
        SE3 T_final;
        double tw_move = 0.0;
        if (!RegisterByThinWall(rough_map2, rough_map1, T_init, T_final, tw_move)) {
            LOG(ERROR) << "[lc] REJECT thinwall " << c.idx1_ << "-" << c.idx2_ << ": registration degenerate";
            c.ndt_score_ = 0;
            return;
        }
        Tw2 = T_final.matrix().cast<float>();
        q = Quatd(T_final.so3().matrix());
        q.normalize();
        t = T_final.translation();
        pcl::transformPointCloud(*rough_map2, *output, Tw2);

        // 统一验收尺子（与 incumbent 完全一致）：cloud 对全分辨率 submap_kf1 的墙中位残差
        const double wall_res = WallResidualOnFullTarget(output, submap_kf1);
        c.wall_residual_ = wall_res;
        if (tw_move > options_.wall_refine_max_move_) {
            LOG(ERROR) << "[lc] REJECT thinwall " << c.idx1_ << "-" << c.idx2_ << ": move " << tw_move
                       << "m > " << options_.wall_refine_max_move_ << "m (residual=" << wall_res << ")";
            c.ndt_score_ = 0;
            return;
        }
        if (wall_res >= options_.wall_refine_thresh_) {
            LOG(ERROR) << "[lc] REJECT thinwall " << c.idx1_ << "-" << c.idx2_ << ": 墙面残差 " << wall_res
                       << " >= 阈值 " << options_.wall_refine_thresh_ << " (move=" << tw_move << "m)";
            c.ndt_score_ = 0;
            return;
        }

        // 评分口径与 GICP getFitnessScore() 等价：output→rough_map1 全部点的最近邻均方距离
        //（无 cutoff，与 499 行 getFitnessScore() 语义一致），1/MSE，过同一 ndt_score_th_ 闸门
        double mse = 1e9;
        {
            pcl::KdTreeFLANN<PointType> score_tree;
            score_tree.setInputCloud(rough_map1);
            std::vector<int> nn_idx(1);
            std::vector<float> nn_sq(1);
            size_t n_used = 0;
            double sq_sum = 0.0;
            for (const auto& pt : output->points) {
                if (!std::isfinite(pt.x)) continue;
                if (score_tree.nearestKSearch(pt, 1, nn_idx, nn_sq) < 1) continue;
                sq_sum += static_cast<double>(nn_sq[0]);
                ++n_used;
            }
            if (n_used > 0) mse = sq_sum / static_cast<double>(n_used);
        }
        c.ndt_score_ = 1.0 / std::max(mse, 1e-6);

        wall_refined_T = T_final;
        wall_refine_done = true;
        const double tw_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tw_start).count();
        LOG(INFO) << "[lc] ACCEPT thinwall " << c.idx1_ << "-" << c.idx2_ << ": 墙面残差 " << wall_res
                  << " move=" << tw_move << "m score=" << c.ndt_score_ << " (" << tw_ms << "ms)";
    } else {
    double fitness = 1e9;
    try {
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setCorrespondenceRandomness(std::min<int>(10, static_cast<int>(rough_map2->size())));
        gicp.setMaximumIterations(options_.gicp_reg_max_iter_);
        gicp.setMaxCorrespondenceDistance(options_.gicp_reg_corr_dist_);
        gicp.setTransformationEpsilon(1e-9);
        gicp.setInputTarget(rough_map1);
        gicp.setInputSource(rough_map2);

        gicp.align(*output, Tw2);
        if (!gicp.hasConverged()) {
            c.ndt_score_ = 0;
            return;
        }
        Tw2 = gicp.getFinalTransformation();
        fitness = gicp.getFitnessScore();
    } catch (...) {
        c.ndt_score_ = 0;
        return;
    }

    if (!Tw2.allFinite()) {
        LOG(ERROR) << "GICP produced invalid transform for candidate " << c.idx1_ << "-" << c.idx2_;
        c.ndt_score_ = 0;
        return;
    }

    // 得分 = 1 / 均方对应距离，越大越好（与 rk100 一致）
    c.ndt_score_ = 1.0 / std::max(fitness, 1e-6);

    if (options_.verbose_) {
        LOG(INFO) << "[lc] gicp " << c.idx1_ << "-" << c.idx2_ << ": fitness=" << fitness
                  << " score=" << c.ndt_score_ << " tgt_pts=" << rough_map1->size()
                  << " src_pts=" << rough_map2->size();
    }

    Mat4d T = Tw2.cast<double>();
    q = Quatd(T.block<3, 3>(0, 0));
    q.normalize();
    t = T.block<3, 1>(0, 3);

    // ===== 墙面中位数精修 =====
    // GICP 最小化全局 RMSE（平方和），对离群点敏感、且地面点在任何水平平移下
    // 距离几乎不变（无约束），走廊场景容易收敛到"墙错开十几厘米"的解。
    // 这里改用墙面残差【中位数】（抗离群）直接优化 (dx,dy,dyaw)。
    // wall_refined_T / wall_refine_done 已在分支外声明（两种模式共用，save 块读取外层变量）
    if (options_.enable_wall_refine_) {
        SE3 Twl_refine(q, t);
        double wall_res = std::numeric_limits<double>::max();
        // 坐标系说明：submap_kf2 是【源帧局部系】、submap_kf1 是【世界系】，
        // 两者不同系，不能直接送进精修（函数内部会对 source 再乘一次 T，造成双重变换）。
        // output 是 GICP 把源子图变换到目标系后的结果，与 submap_kf1 同系，用它才对。
        const bool ok = RefineByWallMedian(output, submap_kf1, Twl_refine, wall_res);
        c.wall_residual_ = wall_res;
        if (options_.verbose_) {
            LOG(INFO) << "[lc][wall] pre-check " << c.idx1_ << "-" << c.idx2_ << ": src_pts="
                      << (output ? output->size() : 0) << " tgt_pts=" << (submap_kf1 ? submap_kf1->size() : 0)
                      << " residual=" << wall_res;
        }

        if (ok) {
            // 【关键】精修结果必须回写到 q/t，否则下面的 Twl2_aligned(q,t) 仍用 GICP 的原始位姿，
            // 精修就白算了（此前就踩过这个坑：refined.pcd 看着对，但进优化的是旧位姿）。
            Tw2 = Twl_refine.matrix().cast<float>();
            const Mat3d R_ref = Twl_refine.so3().matrix();
            q = Quatd(R_ref);
            q.normalize();
            t = Twl_refine.translation();
            LOG(INFO) << "[lc] ACCEPT wall " << c.idx1_ << "-" << c.idx2_ << ": 墙面残差 " << wall_res
                      << " (阈值 " << options_.wall_refine_thresh_ << ", gicp_score=" << c.ndt_score_ << ")";
            wall_refined_T = Twl_refine;
            wall_refine_done = true;
        } else {
            LOG(ERROR) << "[lc] REJECT wall " << c.idx1_ << "-" << c.idx2_ << ": 墙面残差 " << wall_res
                       << " (阈值 " << options_.wall_refine_thresh_ << ", gicp_score=" << c.ndt_score_ << ")";
            c.ndt_score_ = 0;
            return;
        }
    }
    }  // ===== end reg_mode gicp_wall（incumbent） =====

    SE3 Twl2_aligned(q, t);
    SE3 Tl2b2 = kf2->GetOptLidarPose().inverse() * kf2->GetOptBodyPose();
    SE3 Twb2_aligned = Twl2_aligned * Tl2b2;
    c.Tij_ = kf1->GetOptBodyPose().inverse() * Twb2_aligned;

    // LOG(INFO) << "lc result " << c.idx1_ << " -> " << c.idx2_ << ": score=" << c.ndt_score_ << ", Tw2_t="
    //           << t.transpose() << ", Tij_t=" << c.Tij_.translation().transpose() << ", saved=" << prefix;

    // 保存通过全部判据的回环点云（与原实现一致：只有进入优化的候选才落盘）
    if (c.ndt_score_ > options_.ndt_score_th_ && options_.save_loop_clouds_) {
        try {
            const std::string tag = std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_);
            pcl::io::savePCDFileBinaryCompressed("./data/lc_" + tag + "_out.pcd", *output);
            pcl::io::savePCDFileBinaryCompressed("./data/lc_" + tag + "_tgt.pcd", *rough_map1);
            if (wall_refine_done) {
                // 精修后的源点云（目标系），与 tgt 叠加即可核验精修效果
                CloudPtr refined(new PointCloudType);
                pcl::transformPointCloud(*submap_kf2, *refined, wall_refined_T.matrix());
                pcl::io::savePCDFileBinaryCompressed("./data/lc_" + tag + "_refined.pcd", *refined);
            }
        } catch (const std::exception& e) {
            LOG(WARNING) << "save loop clouds failed (debug only, ignored): " << e.what();
        }
    }
}

namespace {

/// 估计点云法向
void EstimateNormals(const CloudPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, int knn = 20) {
    normals.reset(new pcl::PointCloud<pcl::Normal>());
    if (!cloud || cloud->size() < static_cast<size_t>(knn)) {
        return;
    }
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    ne.setKSearch(knn);
    ne.compute(*normals);
}

/// cloud 到 target 的最近邻距离中位数，只统计 target 上法向为竖直面的点
double WallMedianResidual(const CloudPtr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& target_normals,
                          const pcl::KdTreeFLANN<PointType>& kdtree, double nz_thr) {
    if (!cloud || cloud->empty() || !target_normals || target_normals->empty()) {
        return std::numeric_limits<double>::max();
    }

    std::vector<double> wall_d;
    wall_d.reserve(cloud->size() / 2);
    std::vector<int> idx(1);
    std::vector<float> sqd(1);
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x)) continue;
        if (kdtree.nearestKSearch(pt, 1, idx, sqd) < 1) continue;
        const auto& n = target_normals->points[idx[0]];
        if (!std::isfinite(n.normal_z)) continue;
        if (std::fabs(n.normal_z) >= nz_thr) continue;  // 非竖直面
        wall_d.push_back(std::sqrt(static_cast<double>(sqd[0])));
    }

    if (wall_d.size() < 20) {
        return std::numeric_limits<double>::max();
    }
    const size_t mid = wall_d.size() / 2;
    std::nth_element(wall_d.begin(), wall_d.begin() + mid, wall_d.end());
    return wall_d[mid];
}

/// 法向定向 + 可信掩码 om（full_regress.py orient_vec 的 C++ 移植）
/// 地板/天花板点强制 ±z；墙面点（|nz|<0.3 且非地板/天花板）做成对反极性定向
/// （同一面墙两侧观测的法向应互相指向对方），无配对的墙点用自由侧计数定向。
/// om 输出"法向已定向、可参与反极性门控"的掩码（地板/天花板/墙面）。
/// 注意：法向会被就地修改；om 与 cloud 点一一对应。
void OrientNormals(const CloudPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, double floor_z,
                   double ceil_z, std::vector<bool>& om) {
    om.assign(cloud ? cloud->size() : 0, false);
    if (!cloud || !normals || normals->size() != cloud->size()) return;
    const size_t n = cloud->size();

    // 归一化 + 地板/天花板强制 ±z（与 Python：先归一化、后按高度强制一致）
    std::vector<int> wall_idx;
    wall_idx.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        auto& nrm = normals->points[i];
        double nx = nrm.normal_x, ny = nrm.normal_y, nz = nrm.normal_z;
        const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (norm > 1e-12) {
            nx /= norm;
            ny /= norm;
            nz /= norm;
        }
        const double z = cloud->points[i].z;
        const bool is_floor = z < floor_z;
        const bool is_ceil = z > ceil_z;
        if (is_floor) {
            nx = 0.0;
            ny = 0.0;
            nz = 1.0;
        } else if (is_ceil) {
            nx = 0.0;
            ny = 0.0;
            nz = -1.0;
        }
        nrm.normal_x = nx;
        nrm.normal_y = ny;
        nrm.normal_z = nz;
        om[i] = is_floor || is_ceil;
        if (!is_floor && !is_ceil && std::fabs(nz) < 0.3) wall_idx.push_back(static_cast<int>(i));
    }
    const size_t nw = wall_idx.size();
    if (nw <= 10) return;  // 墙点太少不做成对定向（om 只含地板/天花板）

    // 抽取墙面点与其法向
    CloudPtr walls(new PointCloudType);
    walls->points.resize(nw);
    walls->width = static_cast<uint32_t>(nw);
    walls->height = 1;
    walls->is_dense = false;
    std::vector<Vec3d> wpos(nw), nwv(nw);
    for (size_t i = 0; i < nw; ++i) {
        walls->points[i] = cloud->points[wall_idx[i]];
        wpos[i] = Vec3d(walls->points[i].x, walls->points[i].y, walls->points[i].z);
        const auto& nrm = normals->points[wall_idx[i]];
        nwv[i] = Vec3d(nrm.normal_x, nrm.normal_y, nrm.normal_z);
    }

    // 成对反极性定向：k=8 邻域（ub=0.45）内第一个 dots<-0.7 的邻居为 partner，
    // 若 (W_i - W_j)·N_i < 0 则翻转（法向应背离 partner 一侧）
    pcl::KdTreeFLANN<PointType> wtree;
    wtree.setInputCloud(walls);
    std::vector<int> idx(8);
    std::vector<float> sq(8);
    std::vector<int> partner(nw, -1);
    constexpr double kPairR2 = 0.45 * 0.45;
    for (size_t i = 0; i < nw; ++i) {
        const int found = wtree.nearestKSearch(walls->points[i], 8, idx, sq);
        for (int k = 0; k < found; ++k) {
            if (sq[k] > kPairR2) continue;        // distance_upper_bound=0.45
            if (sq[k] < 1e-18) continue;          // 排除自身（dd<1e-9）
            if (nwv[i].dot(nwv[idx[k]]) < -0.7) {
                partner[i] = idx[k];
                break;                            // 取第一个满足的邻居
            }
        }
    }
    for (size_t i = 0; i < nw; ++i) {
        const int j = partner[i];
        if (j < 0) continue;
        if ((wpos[i] - wpos[j]).dot(nwv[i]) < 0) nwv[i] = -nwv[i];
    }

    // 自由侧定向：无配对的墙点，法向指向点更稀疏的一侧（±0.4m 偏移点各数 10 邻域，
    // ub=0.28，排除距墙点自身 <0.1 的邻居）
    pcl::KdTreeFLANN<PointType> atree;
    atree.setInputCloud(cloud);
    std::vector<int> idx2(10);
    std::vector<float> sq2(10);
    constexpr double kFreeR2 = 0.28 * 0.28;
    for (size_t i = 0; i < nw; ++i) {
        if (partner[i] >= 0) continue;
        const Vec3d& w = wpos[i];
        const Vec3d& nn = nwv[i];
        auto cnt_off = [&](const Vec3d& s) -> int {
            PointType q = cloud->points[wall_idx[i]];
            q.x = static_cast<float>(s.x());
            q.y = static_cast<float>(s.y());
            q.z = static_cast<float>(s.z());
            const int f = atree.nearestKSearch(q, 10, idx2, sq2);
            int cnt = 0;
            for (int k = 0; k < f; ++k) {
                if (sq2[k] > kFreeR2) continue;   // distance_upper_bound=0.28
                const auto& nb = cloud->points[idx2[k]];
                const double dx = nb.x - w.x(), dy = nb.y - w.y(), dz = nb.z - w.z();
                if (dx * dx + dy * dy + dz * dz > 0.1 * 0.1) ++cnt;  // 排除墙点自身
            }
            return cnt;
        };
        const int c1 = cnt_off(w + 0.4 * nn);
        const int c2 = cnt_off(w - 0.4 * nn);
        if (c1 != c2 && c1 < c2) nwv[i] = -nwv[i];
    }

    // 写回 + om 标记墙面点
    for (size_t i = 0; i < nw; ++i) {
        auto& nrm = normals->points[wall_idx[i]];
        nrm.normal_x = nwv[i].x();
        nrm.normal_y = nwv[i].y();
        nrm.normal_z = nwv[i].z();
        om[wall_idx[i]] = true;
    }
}

}  // namespace

double LoopClosing::WallResidualOnFullTarget(const CloudPtr& cloud, const CloudPtr& target_full) {
    if (!cloud || !target_full || cloud->empty() || target_full->empty()) {
        return std::numeric_limits<double>::max();
    }
    // 与 incumbent 精修同一把尺子：目标全分辨率点云 20NN 法向 + 源点最近邻距离中位数（墙点）
    pcl::PointCloud<pcl::Normal>::Ptr tn;
    EstimateNormals(target_full, tn, options_.thinwall_knn_);
    if (!tn || tn->size() != target_full->size()) {
        return std::numeric_limits<double>::max();
    }
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(target_full);
    return WallMedianResidual(cloud, tn, tree, options_.wall_normal_z_thr_);
}

/// NVMS 紧核薄墙配准（full_regress.py nvms_tight 的 3-DOF 移植，主循环在 rk100 已验证）。
/// - source：源帧局部系体素云（rough_map2）；target：世界系体素云（rough_map1）
/// - 参数化：T_final = Delta(bx,by,dyaw)·T_init，Delta 绕固定中心 c_w（T_init·src 质心），
///   与 incumbent RefineByWallMedian 的 Delta 合成公式完全一致（z/roll/pitch 锁定，信任轨迹）
/// - 邻域平面拟合 + 反极性/平行度门控 + 紧 Huber IRLS（对应 scipy least_squares(huber, f_scale)）
bool LoopClosing::RegisterByThinWall(const CloudPtr& source, const CloudPtr& target, const SE3& T_init,
                                     SE3& T_final, double& move) {
    move = 0.0;
    T_final = T_init;
    if (!source || !target || source->size() < 50 || target->size() < 50) return false;

    const auto t0 = std::chrono::steady_clock::now();
    const size_t ns = source->size();
    const size_t nt = target->size();

    // 1) 法向估计 + orient_vec 定向（两侧观测反极性配对 / 自由侧计数 / 地板天花板强制）
    pcl::PointCloud<pcl::Normal>::Ptr sn, tn;
    EstimateNormals(source, sn, options_.thinwall_knn_);
    EstimateNormals(target, tn, options_.thinwall_knn_);
    if (!sn || sn->size() != ns || !tn || tn->size() != nt) return false;
    std::vector<bool> s_om, t_om;
    OrientNormals(source, sn, options_.thinwall_floor_z_, options_.thinwall_ceil_z_, s_om);
    OrientNormals(target, tn, options_.thinwall_floor_z_, options_.thinwall_ceil_z_, t_om);

    // 2) 源点世界系初值 p_i = T_init·s_i；固定旋转中心 c_w = mean(p)
    std::vector<Vec3d> p(ns);
    Vec3d c_w = Vec3d::Zero();
    for (size_t i = 0; i < ns; ++i) {
        const auto& s = source->points[i];
        p[i] = T_init * Vec3d(s.x, s.y, s.z);
        c_w += p[i];
    }
    c_w /= static_cast<double>(ns);

    // 3) 目标 KD 树与常量（与 Python 严格一致：k=20, ub=1.2, cos(100°)）
    pcl::KdTreeFLANN<PointType> ttree;
    ttree.setInputCloud(target);
    const int K = options_.thinwall_knn_;
    const double ub2 = options_.thinwall_nn_radius_ * options_.thinwall_nn_radius_;
    constexpr double kCos100 = -0.17364817766693033;  // cos(100°)
    constexpr double kPar = 0.17364817766693033;      // |cos(100°)|
    const double huber_delta = options_.thinwall_huber_delta_;
    std::vector<int> nn_idx(K);
    std::vector<float> nn_sq(K);

    // 4) IRLS 外层：res 调度 1.0/0.75/0.5（it<2/it<3/其余）
    double bx = 0.0, by = 0.0, byaw = 0.0;
    bool solved_any = false;
    int iters_done = 0;
    size_t last_n_use = 0;

    // 每个源点的对应平面缓存（外层迭代内固定，内层 GN 共用）
    std::vector<Vec3d> mu(ns), npv(ns), pp_rel(ns);
    std::vector<double> pw(ns);
    std::vector<char> usable(ns, 0);

    for (int it = 0; it < options_.thinwall_max_outer_; ++it) {
        iters_done = it + 1;
        const double res = it < 2 ? 1.0 : (it < 3 ? 0.75 : 0.5);
        const double cb = std::cos(byaw), sb = std::sin(byaw);

        // ---- 对应搜索 + 逐点平面拟合（mu / n_p / a2D 权重）----
        size_t n_use = 0;
        for (size_t i = 0; i < ns; ++i) {
            usable[i] = 0;
            const Vec3d rel = p[i] - c_w;
            const Vec3d pp(cb * rel.x() - sb * rel.y(), sb * rel.x() + cb * rel.y(), rel.z());
            const Vec3d q = pp + c_w + Vec3d(bx, by, 0.0);
            pp_rel[i] = pp;

            PointType qp = source->points[i];
            qp.x = static_cast<float>(q.x());
            qp.y = static_cast<float>(q.y());
            qp.z = static_cast<float>(q.z());
            const int found = ttree.nearestKSearch(qp, K, nn_idx, nn_sq);

            const Eigen::Vector3f& s_n = sn->points[i].getNormalVector3fMap();
            int cnt = 0;
            Vec3d mu_i = Vec3d::Zero();
            for (int k = 0; k < found; ++k) {
                if (nn_sq[k] > ub2) continue;  // distance_upper_bound
                const int j = nn_idx[k];
                const auto& nb = target->points[j];
                const Eigen::Vector3f& t_n = tn->points[j].getNormalVector3fMap();
                const float dot = s_n.dot(t_n);
                const bool gate = (s_om[i] && t_om[j]) ? (dot > kCos100) : (std::fabs(dot) > kPar);
                if (!gate) continue;
                mu_i += Vec3d(nb.x, nb.y, nb.z);
                ++cnt;
            }
            if (cnt < 4) continue;  // ok = cnt>=4
            mu_i /= static_cast<double>(cnt);

            // 协方差 + 最小特征向量（法向）
            Mat3d cov = Mat3d::Zero();
            for (int k = 0; k < found; ++k) {
                if (nn_sq[k] > ub2) continue;
                const int j = nn_idx[k];
                const Eigen::Vector3f& t_n = tn->points[j].getNormalVector3fMap();
                const float dot = s_n.dot(t_n);
                const bool gate = (s_om[i] && t_om[j]) ? (dot > kCos100) : (std::fabs(dot) > kPar);
                if (!gate) continue;
                const auto& nb = target->points[j];
                const Vec3d d(nb.x - mu_i.x(), nb.y - mu_i.y(), nb.z - mu_i.z());
                cov += d * d.transpose();
            }
            cov /= static_cast<double>(std::max(cnt - 1, 1));

            Eigen::SelfAdjointEigenSolver<Mat3d> es(cov);
            if (es.info() != Eigen::Success) continue;
            const Vec3d ev = es.eigenvalues();                 // 升序
            Vec3d n_p = es.eigenvectors().col(0);              // 最小特征向量
            const double a2d =
                (std::sqrt(std::max(ev[1], 0.0)) - std::sqrt(std::max(ev[0], 0.0))) /
                (std::sqrt(std::max(ev[2], 0.0)) + 1e-12);
            // 与源法向同侧（sgn = sign(n_p·sn)，0 → 1）
            double sgn = n_p.dot(Vec3d(s_n.x(), s_n.y(), s_n.z()));
            if (sgn == 0.0) sgn = 1.0;
            n_p *= sgn > 0 ? 1.0 : -1.0;

            const Vec3d diff = q - mu_i;
            const double r = diff.dot(n_p);
            const double dq = diff.norm();
            const double w = 0.5 * a2d + 0.5 * std::exp(-dq / (res * 4.0));
            if (!std::isfinite(r) || std::fabs(r) >= res) continue;  // use = |r|<res

            mu[i] = mu_i;
            npv[i] = n_p;
            pw[i] = w;
            usable[i] = 1;
            ++n_use;
        }
        last_n_use = n_use;
        if (n_use < 50) break;  // Python: len(Is)<50 → break

        // ---- 内层 GN（≤10 步，对应 scipy least_squares(max_nfev=60)）----
        for (int inner = 0; inner < 10; ++inner) {
            const double cbi = std::cos(byaw), sbi = std::sin(byaw);
            Mat3d H = Mat3d::Zero();
            Vec3d g = Vec3d::Zero();
            for (size_t i = 0; i < ns; ++i) {
                if (!usable[i]) continue;
                const Vec3d& n_p = npv[i];
                // 残差模型 e(δ) = n_p·( Rz(byaw+δyaw)(p_i-c_w) + c_w + [bx+δx, by+δy, 0] - mu_i )
                const Vec3d rel = p[i] - c_w;
                const Vec3d pp(cbi * rel.x() - sbi * rel.y(), sbi * rel.x() + cbi * rel.y(), rel.z());
                const Vec3d q = pp + c_w + Vec3d(bx, by, 0.0);
                const double e = (q - mu[i]).dot(n_p);
                // 雅可比：∂/∂bx → n_p.x；∂/∂by → n_p.y；∂/∂dyaw → n_p·ẑ×pp = (-pp.y, pp.x, 0)
                Eigen::Matrix<double, 1, 3> J;
                J(0) = n_p.x();
                J(1) = n_p.y();
                J(2) = n_p.x() * (-pp.y()) + n_p.y() * pp.x();
                // 紧 Huber 权（f_scale=0.05）：|e|≤δ → 1，否则 δ/|e|
                const double rho = pw[i] * (std::fabs(e) <= huber_delta ? 1.0 : huber_delta / std::fabs(e));
                H += rho * J.transpose() * J;
                g += rho * J.transpose() * e;
            }
            const double damp = 1e-9 * std::max(1.0, H.trace() / 3.0);
            Mat3d Hd = H;
            for (int d = 0; d < 3; ++d) Hd(d, d) += damp;
            const Vec3d delta = -Hd.ldlt().solve(g);
            if (!delta.allFinite()) break;
            bx += delta.x();
            by += delta.y();
            byaw += delta.z();
            solved_any = true;
            if (std::fabs(delta.x()) < 1e-8 && std::fabs(delta.y()) < 1e-8 && std::fabs(delta.z()) < 1e-9) break;
        }
        if (!std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(byaw)) return false;
    }

    if (!solved_any) return false;  // 从未得到可用对应（退化），交由调用方拒绝

    // 5) 合成：T_final = Delta·T_init，Delta 绕 c_w 旋转 dyaw 后平移 (bx,by,0)
    //    与 incumbent RefineByWallMedian 771-776 行完全同一公式
    move = std::hypot(bx, by);
    const double cb = std::cos(byaw), sb = std::sin(byaw);
    Mat3d Rd;
    Rd << cb, -sb, 0.0, sb, cb, 0.0, 0.0, 0.0, 1.0;
    const Vec3d t_delta = c_w + Vec3d(bx, by, 0.0) - Rd * c_w;
    const SE3 Delta(SO3(Rd), t_delta);
    T_final = Delta * T_init;

    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    static double total_ms = 0.0;
    static int total_calls = 0;
    total_ms += ms;
    ++total_calls;
    LOG(INFO) << "[thinwall] src=" << ns << " tgt=" << nt << " iters=" << iters_done << " used=" << last_n_use
              << " delta=(" << bx << "," << by << "," << byaw << ") move=" << move << " " << ms
              << "ms (avg " << total_ms / total_calls << "ms over " << total_calls << " calls)";
    return true;
}

bool LoopClosing::RefineByWallMedian(const CloudPtr& source, const CloudPtr& target, SE3& T, double& residual) {
    residual = std::numeric_limits<double>::max();
    if (!source || !target || source->size() < 50 || target->size() < 50) {
        return false;
    }

    // ===== 性能优化 =====
    // 性能说明：cost() 会被调用约 120 次（3轮 × 3方向 × 10~14次），单次遍历全量源点。
    // 优化点：① 复用点云缓冲区，避免每次 new；② 法向只在目标点云上算一次；
    //         ③ 内联 KD-Tree 查询，不构造中间对象。
    // 不降采样：0.2m 体素会把残差抬高 0.02~0.06m，而回环跑在独立线程、
    //           关键帧约 2s 一个，实测在线 3 倍速零队列溢出，无需牺牲精度。
    const CloudPtr& src_down = source;
    const CloudPtr& tgt_down = target;
    if (src_down->size() < 50 || tgt_down->size() < 50) {
        LOG(WARNING) << "[lc][wall] too few points, skip refine";
        return false;
    }

    pcl::PointCloud<pcl::Normal>::Ptr target_normals;
    EstimateNormals(tgt_down, target_normals, 20);
    if (!target_normals || target_normals->size() != tgt_down->size()) {
        LOG(ERROR) << "[lc][wall] failed to estimate target normals";
        return false;
    }
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(tgt_down);

    const SE3 T0 = T;
    const Mat3d R0 = T0.so3().matrix();
    const Vec3d t0 = T0.translation();

    // 旋转中心取源点云质心
    Vec3d cen = Vec3d::Zero();
    {
        int n = 0;
        for (const auto& pt : src_down->points) {
            if (!std::isfinite(pt.x)) continue;
            cen += Vec3d(pt.x, pt.y, pt.z);
            ++n;
        }
        if (n == 0) return false;
        cen /= n;
    }
    const Vec3d cen_w = R0 * cen + t0;

    // 复用缓冲区：避免每次 cost() 都 new 一份点云
    CloudPtr moved(new PointCloudType());
    moved->points.resize(src_down->size());
    std::vector<double> wall_d;
    wall_d.reserve(src_down->size());
    std::vector<int> knn_idx(1);
    std::vector<float> knn_sq(1);

    // 内联的 cost：直接算中位数，不再构造中间对象
    auto cost = [&](double dx, double dy, double dyaw) -> double {
        const double c = std::cos(dyaw), s = std::sin(dyaw);
        wall_d.clear();
        const size_t N = src_down->size();
        for (size_t i = 0; i < N; ++i) {
            const auto& pt = src_down->points[i];
            const double x = pt.x - cen_w.x();
            const double y = pt.y - cen_w.y();
            PointType q;
            q.x = static_cast<float>(c * x - s * y + cen_w.x() + dx);
            q.y = static_cast<float>(s * x + c * y + cen_w.y() + dy);
            q.z = pt.z;
            q.intensity = pt.intensity;
            moved->points[i] = q;

            if (kdtree.nearestKSearch(q, 1, knn_idx, knn_sq) < 1) continue;
            const auto& n = target_normals->points[knn_idx[0]];
            if (!std::isfinite(n.normal_z)) continue;
            if (std::fabs(n.normal_z) >= options_.wall_normal_z_thr_) continue;
            wall_d.push_back(std::sqrt(static_cast<double>(knn_sq[0])));
        }
        if (wall_d.size() < 20) return std::numeric_limits<double>::max();
        const size_t mid = wall_d.size() / 2;
        std::nth_element(wall_d.begin(), wall_d.begin() + mid, wall_d.end());
        return wall_d[mid];
    };

    double bx = 0.0, by = 0.0, byaw = 0.0;
    double best = cost(0.0, 0.0, 0.0);
    const double start_cost = best;
    if (!std::isfinite(best) || best >= std::numeric_limits<double>::max()) {
        LOG(ERROR) << "[lc][wall] wall residual unavailable (no vertical surfaces in target?)";
        residual = start_cost;
        return false;
    }

    // 坐标下降 + 黄金分割一维搜索
    constexpr double kGr = 0.6180339887498949;
    auto lineSearch = [&](int which, double lo, double hi, int iters) -> void {
        for (int k = 0; k < iters; ++k) {
            const double c1 = lo + (hi - lo) * (1.0 - kGr);
            const double c2 = lo + (hi - lo) * kGr;
            const double f1 = (which == 0) ? cost(c1, by, byaw) : (which == 1) ? cost(bx, c1, byaw) : cost(bx, by, c1);
            const double f2 = (which == 0) ? cost(c2, by, byaw) : (which == 1) ? cost(bx, c2, byaw) : cost(bx, by, c2);
            if (f1 < f2) {
                hi = c2;
                if (f1 < best) {
                    best = f1;
                    if (which == 0) bx = c1;
                    else if (which == 1) by = c1;
                    else byaw = c1;
                }
            } else {
                lo = c1;
                if (f2 < best) {
                    best = f2;
                    if (which == 0) bx = c2;
                    else if (which == 1) by = c2;
                    else byaw = c2;
                }
            }
        }
    };

    for (int round = 0; round < 3; ++round) {
        lineSearch(0, bx - 0.15, bx + 0.15, 12);
        lineSearch(1, by - 0.30, by + 0.30, 14);
        lineSearch(2, byaw - 0.05, byaw + 0.05, 12);
    }

    const double move = std::sqrt(bx * bx + by * by);
    if (move > options_.wall_refine_max_move_) {
        LOG(ERROR) << "[lc][wall] refine move " << move << "m exceeds limit " << options_.wall_refine_max_move_
                   << "m, reject";
        residual = best;
        return false;
    }

    // 合并修正量：T_new = Delta * T0
    const double c = std::cos(byaw), s = std::sin(byaw);
    Mat3d Rd = Mat3d::Identity();
    Rd(0, 0) = c; Rd(0, 1) = -s;
    Rd(1, 0) = s; Rd(1, 1) = c;
    const Vec3d t_delta = cen_w + Vec3d(bx, by, 0.0) - Rd * cen_w;
    SE3 Delta(SO3(Rd), t_delta);
    T = Delta * T0;

    residual = best;
    LOG(INFO) << "[lc][wall] refine " << start_cost << " -> " << best << "  move=" << move << "m (dx=" << bx
              << " dy=" << by << " dyaw=" << byaw * 180.0 / M_PI << "deg)";
    return best < options_.wall_refine_thresh_;
}

void LoopClosing::PoseOptimization() {
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptBodyPose());

    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    /// 上一个关键帧的运动约束
    /// TODO 3D激光最好是跟前面多个帧都有关联

    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOBodyPose().inverse() * cur_kf_->GetLIOBodyPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            optimizer_->AddEdge(e);
        }
    }

    if (options_.with_height_) {
        /// 高度约束
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        Vec3d gravity_dir = -cur_kf_->GetState().grav_.vec_;
        if (gravity_dir.norm() > 1e-6) {
            gravity_dir.normalize();
        } else {
            gravity_dir = Vec3d::UnitZ();
        }
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetGravityDirection(gravity_dir);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e);
    }

    /// 回环的约束
    for (auto& c : candidates_) {
        // 该关键帧可能还没有对应的优化顶点（首次优化、或该帧从未加入图），
        // 此时 GetVertex 返回 nullptr，加入优化会在计算误差时产生 nan。
        auto va = optimizer_->GetVertex(c.idx1_);
        auto vb = optimizer_->GetVertex(c.idx2_);
        if (!va || !vb) {
            LOG(WARNING) << "[lc] skip loop " << c.idx1_ << "-" << c.idx2_ << ": 顶点不存在";
            continue;
        }
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, va);
        e->SetVertex(1, vb);
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);

        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        edge_loops_.emplace_back(e);
    }

    if (optimizer_->GetEdges().empty()) {
        return;
    }

    if (candidates_.empty()) {
        return;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(options_.verbose_);

    // 优化前校验：位姿图的任何 nan 都会在 SO3::exp 内部触发 ensure 失败并 abort，
    // 这里在进入求解器之前把所有非有限值挡住（顶点位姿 / 边测量值）。
    {
        int bad_v = 0, bad_e = 0;
        for (auto &vt : kf_vert_)
        {
            if (!vt) { bad_v++; continue; }
            if (!vt->Estimate().matrix().allFinite())
            {
                LOG(ERROR) << "[lc] 顶点 " << vt->GetId() << " 位姿非有限，重置为 LIO 位姿";
                const int id = vt->GetId();
                if (id >= 0 && id < (int)all_keyframes_.size() && all_keyframes_[id])
                    vt->SetEstimate(all_keyframes_[id]->GetLIOBodyPose());
                else
                    vt->SetEstimate(SE3());
                if (!vt->Estimate().matrix().allFinite()) vt->SetEstimate(SE3());
                bad_v++;
            }
        }
        for (auto &e : edge_loops_)
        {
            if (!e) continue;
            if (!e->GetMeasurement().matrix().allFinite())
            {
                LOG(ERROR) << "[lc] 回环边测量值非有限，降级为无效";
                e->SetLevel(1);
                bad_e++;
            }
        }
        if (bad_v || bad_e)
            LOG(WARNING) << "[lc] 优化前修复非有限值: 顶点 " << bad_v << ", 回环边 " << bad_e;
    }

    const int iters = optimizer_->Optimize(20);

    // 事后离群剔除：用核函数 delta 作为 chi2 上限
    int cnt_outliers = 0;
    for (auto &e : edge_loops_)
    {
        if (!e || e->GetRobustKernel() == nullptr) continue;
        if (e->Chi2() > e->GetRobustKernel()->Delta())
        {
            e->SetLevel(1);
            cnt_outliers++;
        }
        else
        {
            e->SetRobustKernel(nullptr);
        }
    }

    if (options_.verbose_)
    {
        LOG(INFO) << "[lc] optimize: iters=" << iters << "/20 "
                  << (iters >= 20 ? "NOT_CONVERGED" : "converged")
                  << ", chi2_robust=" << optimizer_->ActiveRobustChi2()
                  << ", loops=" << edge_loops_.size() << ", outliers=" << cnt_outliers;
    }

    /// get results
    for (auto &vert : kf_vert_)
    {
        if (!vert) continue;
        SE3 pose = vert->Estimate();
        // 优化发散时 Estimate() 可能为 nan，写回会污染整个位姿图与地图，
        // 这里直接丢弃该顶点的更新（保留优化前的位姿）。
        if (!pose.matrix().allFinite())
        {
            LOG(ERROR) << "[lc] 顶点 " << vert->GetId() << " 优化结果为非有限值，保持原位姿";
            continue;
        }
        const int vid = vert->GetId();
        if (vid < 0 || vid >= (int)all_keyframes_.size()) continue;
        auto kf_ptr = all_keyframes_[vid];
        if (!kf_ptr) continue;
        kf_ptr->SetOptBodyPose(pose);
    }

    // 记录本次触发区域，供后续空间去重使用
    if (options_.suppress_radius_ > 1e-6 && !candidates_.empty())
    {
        LoopRegion r;
        r.center = cur_kf_->GetOptBodyPose().translation();
        r.stamp = cur_kf_->GetState().timestamp_;
        r.kf_id = cur_kf_->GetID();
        recent_loop_regions_.emplace_back(r);

        // 只保留最近 50 个，防止长时间运行无限增长
        if (recent_loop_regions_.size() > 50)
        {
            recent_loop_regions_.erase(recent_loop_regions_.begin());
        }
    }

    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptBodyPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOBodyPose().translation().transpose();
}

}  // namespace lightning
