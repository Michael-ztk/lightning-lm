//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <unordered_set>
#include <cmath>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

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
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);

    SE3 Twl2_aligned(q, t);
    SE3 Tl2b2 = kf2->GetOptLidarPose().inverse() * kf2->GetOptBodyPose();
    SE3 Twb2_aligned = Twl2_aligned * Tl2b2;
    c.Tij_ = kf1->GetOptBodyPose().inverse() * Twb2_aligned;

    // LOG(INFO) << "lc result " << c.idx1_ << " -> " << c.idx2_ << ": score=" << c.ndt_score_ << ", Tw2_t="
    //           << t.transpose() << ", Tij_t=" << c.Tij_.translation().transpose() << ", saved=" << prefix;

    if (c.ndt_score_ > options_.ndt_score_th_ && options_.save_loop_clouds_) {
        // 调试落盘失败（目录不存在/无写权限/磁盘满）不应中断回环流程：
        // PCL 的 savePCDFile* 在打不开文件时抛 pcl::IOException，未捕获会直接 terminate
        // 整个进程（实测 cwd 不在工程目录时必现）。
        try {
            pcl::io::savePCDFileBinaryCompressed(
                "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
            pcl::io::savePCDFileBinaryCompressed(
                "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
        } catch (const std::exception& e) {
            LOG(WARNING) << "save loop clouds failed (debug only, ignored): " << e.what();
        }
    }
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
