#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <unordered_map>
#include <fstream>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

bool LaserMapping::Init(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, 23, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.wheelspeed_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) {
        WheelSpeedModel(s, obs);
    };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;
    Vec3d lidar_T_wrt_IMU;
    Mat3d lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();
        extrinsic_est_en_ = yaml["fasterlio"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        float height_max = yaml["roi"]["height_max"].as<float>();
        float height_min = yaml["roi"]["height_min"].as<float>();

        preprocess_->SetHeightROI(height_max, height_min);

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;

        /// 射线free-space清洗参数（缺省时使用默认值）
        auto flio = yaml["fasterlio"];
        rayclean_en_ = flio["rayclean_en"] ? flio["rayclean_en"].as<bool>() : rayclean_en_;
        rayclean_voxel_size_ = flio["rayclean_voxel_size"] ? flio["rayclean_voxel_size"].as<double>() : rayclean_voxel_size_;
        rayclean_pass_th_ = flio["rayclean_pass_th"] ? flio["rayclean_pass_th"].as<int>() : rayclean_pass_th_;
        rayclean_end_th_ = flio["rayclean_end_th"] ? flio["rayclean_end_th"].as<int>() : rayclean_end_th_;
        rayclean_plane_protect_dist_ =
            flio["rayclean_plane_protect_dist"] ? flio["rayclean_plane_protect_dist"].as<double>() : rayclean_plane_protect_dist_;
        rayclean_plane_tile_size_ =
            flio["rayclean_plane_tile_size"] ? flio["rayclean_plane_tile_size"].as<double>() : rayclean_plane_tile_size_;
        rayclean_plane_ransac_dist_ =
            flio["rayclean_plane_ransac_dist"] ? flio["rayclean_plane_ransac_dist"].as<double>() : rayclean_plane_ransac_dist_;
        rayclean_plane_min_inlier_frac_ = flio["rayclean_plane_min_inlier_frac"]
                                              ? flio["rayclean_plane_min_inlier_frac"].as<double>()
                                              : rayclean_plane_min_inlier_frac_;
        rayclean_grow_en_ = flio["rayclean_grow_en"] ? flio["rayclean_grow_en"].as<bool>() : rayclean_grow_en_;
        rayclean_grow_end_th_ = flio["rayclean_grow_end_th"] ? flio["rayclean_grow_end_th"].as<int>() : rayclean_grow_end_th_;
        rayclean_grow_steps_ = flio["rayclean_grow_steps"] ? flio["rayclean_grow_steps"].as<int>() : rayclean_grow_steps_;
        rayclean_late_pass_th_ = flio["rayclean_late_pass_th"] ? flio["rayclean_late_pass_th"].as<int>() : rayclean_late_pass_th_;
        rayclean_min_cluster_points_ =
            flio["rayclean_min_cluster_points"] ? flio["rayclean_min_cluster_points"].as<int>() : rayclean_min_cluster_points_;
        rayclean_max_range_ = flio["rayclean_max_range"] ? flio["rayclean_max_range"].as<double>() : rayclean_max_range_;
        rayclean_ray_stride_ = flio["rayclean_ray_stride"] ? flio["rayclean_ray_stride"].as<int>() : rayclean_ray_stride_;
        rayclean_guard_dist_ = flio["rayclean_guard_dist"] ? flio["rayclean_guard_dist"].as<double>() : rayclean_guard_dist_;
        rayclean_iso_remove_ = flio["rayclean_iso_remove"] ? flio["rayclean_iso_remove"].as<bool>() : rayclean_iso_remove_;
        rayclean_debug_save_ = flio["rayclean_debug_save"] ? flio["rayclean_debug_save"].as<bool>() : rayclean_debug_save_;

        if (rayclean_voxel_size_ <= 0.05) {
            LOG(WARNING) << "invalid rayclean_voxel_size=" << rayclean_voxel_size_ << ", use default 0.2";
            rayclean_voxel_size_ = 0.2;
        }
        if (rayclean_ray_stride_ < 1) {
            LOG(WARNING) << "invalid rayclean_ray_stride=" << rayclean_ray_stride_ << ", use 1";
            rayclean_ray_stride_ = 1;
        }
        if (rayclean_min_cluster_points_ < 1) {
            LOG(WARNING) << "invalid rayclean_min_cluster_points=" << rayclean_min_cluster_points_ << ", use default 20";
            rayclean_min_cluster_points_ = 20;
        }
        if (rayclean_max_range_ < 5.0) {
            LOG(WARNING) << "invalid rayclean_max_range=" << rayclean_max_range_ << ", use default 20.0";
            rayclean_max_range_ = 20.0;
        }

        /// 轮速里程计融合参数（缺省时使用默认值）
        odom_en_ = flio["odom_en"] ? flio["odom_en"].as<bool>() : odom_en_;
        odom_vel_noise_ = flio["odom_vel_noise"] ? flio["odom_vel_noise"].as<double>() : odom_vel_noise_;
        odom_pos_noise_floor_ =
            flio["odom_pos_noise_floor"] ? flio["odom_pos_noise_floor"].as<double>() : odom_pos_noise_floor_;
        odom_pos_noise_ratio_ =
            flio["odom_pos_noise_ratio"] ? flio["odom_pos_noise_ratio"].as<double>() : odom_pos_noise_ratio_;
        odom_max_time_diff_ =
            flio["odom_max_time_diff"] ? flio["odom_max_time_diff"].as<double>() : odom_max_time_diff_;
        odom_yaw_offset_ = flio["odom_yaw_offset"] ? flio["odom_yaw_offset"].as<double>() : odom_yaw_offset_;
        // 底盘前进方向在 body 系里的单位向量：底盘系相对 body 系绕 z 转 odom_yaw_offset_
        // 默认 0 时前进 = body +x；安装朝向 y后x左 时应配 -π/2（前进 = body -y）
        fwd_body_ = Vec3d(std::cos(odom_yaw_offset_), std::sin(odom_yaw_offset_), 0.0);
        if (odom_vel_noise_ <= 1e-4) {
            LOG(WARNING) << "invalid odom_vel_noise=" << odom_vel_noise_ << ", use default 0.01";
            odom_vel_noise_ = 0.01;
        }
        if (odom_pos_noise_floor_ <= 1e-4) {
            LOG(WARNING) << "invalid odom_pos_noise_floor=" << odom_pos_noise_floor_ << ", use default 0.002";
            odom_pos_noise_floor_ = 0.002;
        }

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LOG(INFO) << "Using RoboSense Lidar";
    } else if (lidar_type == 5) {
        preprocess_->SetLidarType(LidarType::HESAI);
        LOG(INFO) << "Using Hesai Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    lidar_T_wrt_IMU = math::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));

    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    UL lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        /// 更新最新imu状态
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);

        // LOG(INFO) << "newest wrt lidar: " << timestamp - kf_.GetX().timestamp_;

        /// 更新ui
        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
    }

    last_timestamp_imu_ = timestamp;

    imu_buffer_.emplace_back(imu);
}

void LaserMapping::ProcessOdom(const OdomPtr &odom) {
    UL lock(mtx_buffer_);
    if (odom->timestamp_ < last_timestamp_odom_) {
        LOG(WARNING) << "odom loop back, drop this message";
        return;
    }

    last_timestamp_odom_ = odom->timestamp_;
    odom_buffer_.emplace_back(odom);
    while (odom_buffer_.size() > 500) {
        odom_buffer_.pop_front();
    }
}

bool LaserMapping::Run() {
    if (!SyncPackages()) {
        LOG(WARNING) << "sync package failed";
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    /// the first scan
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetLidarPose());
            }

            return false;
        }
    }

    // LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
    //           << ", end: " << measures_.lidar_end_time_;

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }

    // if (cur_pts < (scan_undistort_->size() * 0.1)) {
    //     /// 降采样太狠了,有效点数不够，用

    //     auto v = voxel_scan_;
    //     v.setLeafSize(0.1, 0.1, 0.1);
    //     v.setInputCloud(scan_undistort_);
    //     v.filter(*scan_down_body_);
    //     cur_pts = scan_down_body_->size();
    // }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    Timer::Evaluate(
        [&, this]() {
            // 成员变量预分配
            residuals_.resize(cur_pts, 0);
            point_selected_surf_.resize(cur_pts, 1);
            plane_coef_.resize(cur_pts, Vec4f::Zero());

            auto old_state = kf_.GetX();

            kf_.Update(ESKF::ObsType::LIDAR, 1e-3);
            state_point_ = kf_.GetX();

            // 轮速序贯紧耦合更新：雷达更新后，用body系线速度约束状态速度。
            // 走廊等退化方向上雷达观测弱约束，轮速直接顶住该方向，防止位姿冻结/漂移
            if (odom_en_) {
                has_wheel_obs_ = false;
                if (!measures_.odom_.empty()) {
                    // 取时间最接近帧尾的轮速观测
                    const OdomPtr &odom = measures_.odom_.back();
                    double dt_odom = std::fabs(odom->timestamp_ - measures_.lidar_end_time_);
                    if (dt_odom <= odom_max_time_diff_) {
                        // 底盘只给前进速度 linear.x，前进方向在 body 系是 fwd_body_（由 odom_yaw_offset 派生）
                        cur_wheel_vel_ = fwd_body_ * odom->linear.x();
                        has_wheel_obs_ = true;

                        // 位移增量基准 last_wheel_pos_ 是上一帧做完更新后的位置，时间基准是上一帧
                        // 帧尾 lidar_end_time_，因此 dt 必须用帧尾时间差，不能再用 odom 戳差，
                        // 否则 Δp 覆盖一个 lidar 帧周期、期望位移却只覆盖 odom 间隔，残差带系统偏置
                        wheel_obs_dt_ = (last_wheel_time_ > 0) ? (measures_.lidar_end_time_ - last_wheel_time_) : 0.0;

                        // 位移观测噪声：σ_pos² = σ_vel²·dt² + (k·|Δd|)²，再取下限
                        //  - σ_vel²·dt²：速度噪声随积分时间传播（主导项）
                        //  - (k·|Δd|)²：比例噪声，模型化轮径标度误差与打滑（误差随位移放大）
                        //  - 下限：仅数值兜底（防止 dt→0 时方差为 0），取值应远小于基础项
                        const double disp = std::fabs(cur_wheel_vel_.dot(fwd_body_)) * wheel_obs_dt_;
                        const double pos_noise_base = odom_vel_noise_ * wheel_obs_dt_ * wheel_obs_dt_;
                        const double pos_noise_scale = std::pow(odom_pos_noise_ratio_ * disp, 2);
                        double pos_noise = std::max(pos_noise_base + pos_noise_scale,
                                                    odom_pos_noise_floor_ * odom_pos_noise_floor_);
                        const double R_wheel = (has_last_wheel_ && wheel_obs_dt_ > 1e-3) ? pos_noise : odom_vel_noise_;

                        kf_.Update(ESKF::ObsType::WHEEL_SPEED, R_wheel);
                        state_point_ = kf_.GetX();

                        // 用本帧融合后的位置作为下一帧增量基准（必须取更新后的 state_point_）
                        last_wheel_pos_ = state_point_.pos_;
                        last_wheel_time_ = measures_.lidar_end_time_;
                        has_last_wheel_ = true;

                        static int odom_log_cnt = 0;
                        if (odom_log_cnt++ % 100 == 0) {
                            Vec3d v_w = state_point_.vel_;
                            LOG(INFO) << "wheel odom fused: dt=" << dt_odom << "s obs_dt=" << wheel_obs_dt_
                                      << "s v_meas=" << cur_wheel_vel_.dot(fwd_body_) << "m/s |v_state|=" << v_w.norm()
                                      << "m/s mode=" << ((has_last_wheel_ && wheel_obs_dt_ > 1e-3) ? "delta" : "vel")
                                      << " R=" << R_wheel;
                        }
                    } else {
                        static int warn_cnt = 0;
                        if (warn_cnt++ % 100 == 0) {
                            LOG(WARNING) << "wheel odom too old: dt=" << dt_odom << "s, discard";
                        }
                    }
                }
            }

            if (keep_first_imu_estimation_ && all_keyframes_.size() < 5 &&
                (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180) {
                kf_.ChangeX(old_state);
                state_point_ = old_state;

                LOG(INFO) << "set state as prediction";
            }

            //SE3 delta = old_state.GetPose().inverse() * state_point_.GetPose();
            //LOG(INFO) << "delta norm: " << delta.translation().norm() << ", " << delta.so3().log().norm() * 180 / M_PI;

            // LOG(INFO) << "old yaw: " << old_state.rot_.angleZ() << ", new: " << state_point_.rot_.angleZ();

            state_point_.timestamp_ = measures_.lidar_end_time_;
            euler_cur_ = state_point_.rot_;
            pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    // LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
    //           << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    /// keyframes
    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOBodyPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        }
    }

    /// 更新kf_for_imu
    kf_imu_ = kf_;
    if (!measures_.imu_.empty()) {
        double t = measures_.imu_.back()->timestamp;
        for (auto &imu : imu_buffer_) {
            double dt = imu->timestamp - t;
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
            t = imu->timestamp;
        }
    }

    if (ui_) {
        ui_->UpdateScan(scan_undistort_, state_point_.GetLidarPose());
    }

    return true;
}

void LaserMapping::MakeKF() {
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    if (last_kf_) {
        // LOG(INFO) << "last kf lio: " << last_kf_->GetLIOPose().translation().transpose()
        //           << ", opt: " << last_kf_->GetOptPose().translation().transpose();

        SE3 delta = last_kf_->GetLIOBodyPose().inverse() * kf->GetLIOBodyPose();
        kf->SetOptBodyPose(last_kf_->GetOptBodyPose() * delta);
    } else {
        kf->SetOptBodyPose(kf->GetLIOBodyPose());
    }

    kf->SetState(state_point_);

    // LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
    //           << ", lio_body: " << kf->GetLIOBodyPose().translation().transpose()
    //           << ", lio_lidar: " << kf->GetLIOLidarPose().translation().transpose()
    //           << ", opt_body: " << kf->GetOptBodyPose().translation().transpose()
    //           << ", opt_lidar: " << kf->GetOptLidarPose().translation().transpose() << ", time: "
    //           << std::setprecision(14) << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        UL lock(mtx_keyframes_);
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    // 有keyframes时更新local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            // 优先使用雷达预处理设置的传感器时间戳（如Hesai），否则回退到ROS header时间戳
            double timestamp = cloud->header.stamp != 0
                                   ? double(cloud->header.stamp) * 1e-9
                                   : ToSec(msg->header.stamp);

            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            // LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
            //           << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        LOG(INFO) << "lidar or imu is empty";
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);

            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                /// timestamp 有异常
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_ * 1.1;

        // LOG(INFO) << "recompute lidar end time: " << std::setprecision(14) << lidar_end_time_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        LOG(INFO) << "sync failed: " << std::setprecision(14) << last_timestamp_imu_ << ", " << lidar_end_time_;
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    /*** push odom data within [lidar_begin_time_, lidar_end_time_] ***/
    measures_.odom_.clear();
    while (!odom_buffer_.empty() && odom_buffer_.front()->timestamp_ < measures_.lidar_begin_time_) {
        odom_buffer_.pop_front();  // 过旧的直接丢弃
    }
    while (!odom_buffer_.empty() && odom_buffer_.front()->timestamp_ <= lidar_end_time_) {
        measures_.odom_.emplace_back(odom_buffer_.front());
        odom_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            Mat3f R_wl = (s.rot_ * s.offset_R_lidar_).matrix().cast<float>();
            Vec3f t_wl = (s.rot_ * s.offset_t_lidar_ + s.pos_).cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                points_near.clear();

                /** Find the closest surfaces in the map **/
                // if (obs.converge_) {
                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                if (point_selected_surf_[i]) {
                    point_selected_surf_[i] =
                        math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    } else {
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        obs.valid_ = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            obs.h_x_ = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            obs.residual_.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const Mat3f off_R = s.offset_R_lidar_.matrix().cast<float>();
            const Vec3f off_t = s.offset_t_lidar_.cast<float>();
            const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                Vec3f point_this_be = corr_pts_[i].head<3>();
                Mat3f point_be_crossmat = math::SKEW_SYM_MATRIX(point_this_be);
                Vec3f point_this = off_R * point_this_be + off_t;
                Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                Vec3f norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                Vec3f C(Rt * norm_vec);
                Vec3f A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    Vec3f B(point_be_crossmat * off_R.transpose() * C);
                    obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0], B[1],
                        B[2], C[0], C[1], C[2];
                } else {
                    obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0;
                }

                /// 增加了cauchy's robust kernel
                float res = -corr_pts_[i][3];
                float rho, drho;

                const float delta = 2.0;
                const float dsqr = delta * delta;
                const float dsqr_inv = 1.0 / dsqr;

                if (res >= 0) {
                    rho = dsqr * std::log(1 + res * dsqr_inv);
                    drho = 1.0 / (1 + res * dsqr_inv);
                } else {
                    rho = -dsqr * std::log(1 - res * dsqr_inv);
                    drho = 1.0 / (1 - res * dsqr_inv);
                }

                obs.residual_(i) = rho;
                obs.h_x_.block<1, 12>(i, 0) = obs.h_x_.block<1, 12>(i, 0).eval() * drho;

                // obs.residual_(i) = res;
            });
        },
        "    ObsModel (IEKF Build Jacobian)");

    /// 填入中位数平方误差
    std::vector<double> res_sq2;
    for (size_t i = 0; i < cnt_pts; ++i) {
        if (point_selected_surf_[i]) {
            double r = residuals_[i];
            res_sq2.emplace_back(r * r);
        }
    }

    if (!res_sq2.empty()) {
        std::sort(res_sq2.begin(), res_sq2.end());
        obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
        obs.lidar_residual_max_ = res_sq2[res_sq2.size() - 1];
        // LOG(INFO) << "residual mean: " << obs.lidar_residual_mean_ << ", max: " << obs.lidar_residual_max_
        //           << ", 85%: " << res_sq2[res_sq2.size() * 0.85];
    }
}

/**
 * 轮速观测模型（序贯紧耦合）—— 2 行：位移增量 + 前向速度
 *
 * 约定与 LIDAR 点面残差一致：residual = z - h(x)，H = ∂h/∂x。
 *
 * 行0（位移增量，约束位置块 idx 0）：
 *   观测 z0 = v_body·dt（轮速推算位移），h0 = e_forwardᵀ·(p_cur - p_prev)（状态实际前向位移）
 *   r0 = v_body·dt - e_forwardᵀ·Δp
 *   ∂h0/∂δp = e_forwardᵀ；∂h0/∂δθ = -Δpᵀ·R·hat(fwd_body)
 *   作用：雷达在退化方向钉死绝对位置时直接推动位置，打破冻结
 *
 * 行1（前向速度，约束速度块 idx 12）：
 *   观测 z1 = v_body（轮速），h1 = e_forwardᵀ·v_world（状态速度的前向分量）
 *   残差按 ×dt 换算到米，与行0同量纲共用标量噪声 R
 *   r1 = dt·(v_body - e_forwardᵀ·v_world)
 *   ∂h1/∂δv = dt·e_forwardᵀ；∂h1/∂δθ = dt·(fwd_body × v_body)
 *   作用：走廊退化方向雷达看不到前进速度时直接顶住速度，防止速度状态塌陷
 *
 * 关键：e_forward = R·fwd_body，fwd_body 由 odom_yaw_offset 派生（安装朝向 y后x左 时
 *   为 -π/2，即前进 = body -y）；行1 的旋转雅可比在纯直行时恒为 0，不会把侧向/垂向
 *   速度压成 0 而耦合 yaw（旧 3D 速度模型的 49°突跳根源）。
 *
 * 首帧无上一帧位姿时 valid_=false，跳过本次观测更新，下一帧起才有位移增量基准。
 */
void LaserMapping::WheelSpeedModel(NavState &s, ESKF::CustomObservationModel &obs) {
    if (!has_last_wheel_ || wheel_obs_dt_ <= 1e-3) {
        obs.valid_ = false;  // 无上一帧增量基准，跳过本次轮速观测
        return;
    }

    const int obs_dim = 2;
    obs.h_x_ = Eigen::MatrixXd::Zero(obs_dim, NavState::dim);

    const Mat3d R = s.rot_.matrix();
    const Vec3d e_forward = R * fwd_body_;            // 车头方向（body 前进轴在世界系）
    const Vec3d v_w = s.vel_;                         // 世界系速度
    const Vec3d v_body = R.transpose() * v_w;         // body 系速度
    const Vec3d delta_state = s.pos_ - last_wheel_pos_;
    const double vx = cur_wheel_vel_.dot(fwd_body_);  // 轮速前向分量（标量）
    const double dt = wheel_obs_dt_;

    // 行0：位移增量残差 = 轮速位移 - 状态实际前向位移
    const double measured_disp = e_forward.dot(delta_state);
    const double expect_disp = vx * dt;

    // 行1：前向速度残差（×dt 换算到米）= 轮速位移 - 状态速度前向分量·dt
    const double measured_v_disp = dt * e_forward.dot(v_w);

    obs.residual_.resize(obs_dim);
    obs.residual_(0) = expect_disp - measured_disp;
    obs.residual_(1) = expect_disp - measured_v_disp;

    // 行0 雅可比
    obs.h_x_.block<1, 3>(0, 0) = e_forward.transpose();                             // ∂h0/∂δp
    obs.h_x_.block<1, 3>(0, 3) = -delta_state.transpose() * R * SO3::hat(fwd_body_);  // ∂h0/∂δθ

    // 行1 雅可比
    obs.h_x_.block<1, 3>(1, 12) = (dt * e_forward).transpose();  // ∂h1/∂δv
    Vec3d dv_dtheta = fwd_body_.cross(v_body);                   // ∂h1/∂δθ
    obs.h_x_.block<1, 3>(1, 3) = (dt * dv_dtheta).transpose();   // ∂h1/∂δθ

    // 供日志/统计用
    obs.lidar_residual_mean_ = std::fabs(obs.residual_(0)) + std::fabs(obs.residual_(1));
    obs.lidar_residual_max_ = std::max(std::fabs(obs.residual_(0)), std::fabs(obs.residual_(1)));
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);
    std::vector<Keyframe::Ptr> keyframes;
    {
        UL lock(mtx_keyframes_);
        keyframes = all_keyframes_;
    }

    for (const auto& kf : keyframes) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(res, res, res);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOLidarPose().matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptLidarPose().matrix());
        }

        *global_map += *cloud_trans;

        // LOG(INFO) << "kf " << kf->GetID() << ", pose: " << kf->GetOptLidarPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    if (!all_keyframes_.empty()) {
        auto start_kf = all_keyframes_.front();
        auto end_kf = all_keyframes_.back();

        Vec3d start_pos = start_kf->GetOptLidarPose().translation();
        Vec3d end_pos = end_kf->GetOptLidarPose().translation();
        Vec3d delta = end_pos - start_pos;

        Vec3d gravity_axis = -start_kf->GetState().grav_.vec_;
        double gravity_norm = gravity_axis.norm();
        double drift_along_gravity = 0.0;
        double drift_orthogonal_gravity = 0.0;

        if (gravity_norm > 1e-6) {
            gravity_axis /= gravity_norm;
            drift_along_gravity = delta.dot(gravity_axis);
            drift_orthogonal_gravity = (delta - drift_along_gravity * gravity_axis).norm();
        }

        // LOG(INFO) << "lidar trajectory start: " << start_pos.transpose();
        // LOG(INFO) << "lidar trajectory end: " << end_pos.transpose();
        // LOG(INFO) << "lidar trajectory gravity axis: " << gravity_axis.transpose();
        // LOG(INFO) << "lidar trajectory drift along gravity: " << drift_along_gravity;
        // LOG(INFO) << "lidar trajectory drift orthogonal gravity: " << drift_orthogonal_gravity;
    }

    // LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

void LaserMapping::RemoveDynamicByKeyframeRays() {
    auto t_begin = std::chrono::steady_clock::now();

    std::vector<Keyframe::Ptr> kfs;
    {
        UL lock(mtx_keyframes_);
        kfs = all_keyframes_;
    }

    if (!rayclean_en_ || kfs.size() < 4) {
        LOG(WARNING) << "ray clean skipped: en=" << (int)rayclean_en_ << " keyframes=" << kfs.size();
        return;
    }

    // 与 GetGlobalMap 完全一致的位姿约定：OptLidarPose 直接作用于关键帧点云
    // （无回环时 OptLidarPose==LIOLidarPose；回环优化后 OptLidarPose 为最终位姿）
    const int n_kf = static_cast<int>(kfs.size());
    const double voxel = rayclean_voxel_size_;
    const double max_range_sq = rayclean_max_range_ * rayclean_max_range_;
    const double min_range = 0.5;  // 过短的射线无意义且易受机器人本体干扰
    const double guard_dist_sq = rayclean_guard_dist_ * rayclean_guard_dist_;

    // 每关键帧预取位姿，避免热循环里反复加锁
    std::vector<Mat3d> kf_rot(n_kf);
    std::vector<Vec3d> kf_pos(n_kf);
    for (int i = 0; i < n_kf; ++i) {
        auto T = kfs[i]->GetOptLidarPose().matrix();
        kf_rot[i] = T.block<3, 3>(0, 0);
        kf_pos[i] = T.block<3, 1>(0, 3);
    }

    /// ---- 第0遍：统计有效点的包围盒（只统计 max_range 内的点，防远处离群点撑爆栅格）----
    Vec3d box_min = Vec3d::Constant(1e18), box_max = Vec3d::Constant(-1e18);
    size_t total_pts = 0;
    for (int i = 0; i < n_kf; ++i) {
        const auto &cloud = kfs[i]->GetCloud();
        if (!cloud) {
            continue;
        }
        total_pts += cloud->size();
        for (const auto &pt : cloud->points) {
            Vec3d pw = kf_rot[i] * pt.getVector3fMap().cast<double>() + kf_pos[i];
            if ((pw - kf_pos[i]).squaredNorm() > max_range_sq) {
                continue;
            }
            box_min = box_min.cwiseMin(pw);
            box_max = box_max.cwiseMax(pw);
        }
        box_min = box_min.cwiseMin(kf_pos[i]);
        box_max = box_max.cwiseMax(kf_pos[i]);
    }
    if (total_pts < 1000) {
        LOG(WARNING) << "ray clean skipped: too few points (" << total_pts << ")";
        return;
    }

    /// ---- 栅格：内存超限时自动放粗体素（密集数组，无哈希开销）----
    double voxel_used = voxel;
    int64_t nx = 0, ny = 0, nz = 0;
    const int64_t kMaxCells = 24 * 1024 * 1024;  // 约240MB峰值内存上限
    while (true) {
        nx = static_cast<int64_t>(std::ceil((box_max.x() - box_min.x()) / voxel_used)) + 2;
        ny = static_cast<int64_t>(std::ceil((box_max.y() - box_min.y()) / voxel_used)) + 2;
        nz = static_cast<int64_t>(std::ceil((box_max.z() - box_min.z()) / voxel_used)) + 2;
        if (nx * ny * nz <= kMaxCells || voxel_used > 2.0) {
            break;
        }
        voxel_used *= 2.0;
        LOG(WARNING) << "ray clean grid too large, coarsen voxel to " << voxel_used << " m";
    }
    if (nx * ny * nz > kMaxCells) {
        LOG(WARNING) << "ray clean skipped: map extent too large (cells=" << (long long)(nx * ny * nz) << ")";
        return;
    }

    // 体素状态打包为8字节原子量，支持多线程无锁更新：
    // [pass_cnt:16][end_cnt:16][last_pass_kf:16][last_end_kf:16]
    // pass_cnt: 不同关键帧射线穿越次数；end_cnt: 不同关键帧端点(表面观测)次数
    const int64_t n_cells = nx * ny * nz;
    std::vector<std::atomic<uint64_t>> cells(n_cells);
    for (int64_t i = 0; i < n_cells; ++i) {
        cells[i].store(0);
    }
    // 每体素点数（无去重），用于孤立噪点检查
    std::vector<std::atomic<uint16_t>> pt_cnt(n_cells);
    for (int64_t i = 0; i < n_cells; ++i) {
        pt_cnt[i].store(0);
    }

    // 世界坐标 -> 栅格索引
    const Vec3d grid_min = box_min - Vec3d(voxel_used, voxel_used, voxel_used);
    const double inv_voxel = 1.0 / voxel_used;
    auto vox_index = [&](const Vec3d &p, int64_t &ix, int64_t &iy, int64_t &iz) -> bool {
        ix = static_cast<int64_t>(std::floor((p.x() - grid_min.x()) * inv_voxel));
        iy = static_cast<int64_t>(std::floor((p.y() - grid_min.y()) * inv_voxel));
        iz = static_cast<int64_t>(std::floor((p.z() - grid_min.z()) * inv_voxel));
        return ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz;
    };

    /// ---- 平面保护：分块RANSAC检测地面/墙面等结构性平面，邻近体素不参与清洗 ----
    // 地面/墙面的体素会因掠射穿越+位姿漂移呈现"被穿越且鲜被观测"的动态特征，是误删的主要来源。
    // 按瓦片分块拟合局部平面（瓦片内局部刚性，漂移弯曲的结构面也能覆盖），保护后不再误删
    std::vector<Eigen::Vector4d> protect_planes;  // (nx,ny,nz,d): n*x+d=0
    {
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(total_pts / 8 + 16);
        for (int i = 0; i < n_kf; ++i) {
            const auto &cloud = kfs[i]->GetCloud();
            if (!cloud) {
                continue;
            }
            for (size_t pi = 0; pi < cloud->points.size(); pi += 8) {
                Vec3d pw = kf_rot[i] * cloud->points[pi].getVector3fMap().cast<double>() + kf_pos[i];
                if ((pw - kf_pos[i]).squaredNorm() > max_range_sq) {
                    continue;
                }
                pts.emplace_back(pw);
            }
        }
        const double tile = rayclean_plane_tile_size_;
        std::unordered_map<uint64_t, std::vector<int>> tiles;
        for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
            const int64_t tx = static_cast<int64_t>(std::floor(pts[i].x() / tile));
            const int64_t ty = static_cast<int64_t>(std::floor(pts[i].y() / tile));
            tiles[(static_cast<uint64_t>(tx) << 32) ^ static_cast<uint64_t>(ty & 0xFFFFFFFFll)].push_back(i);
        }
        const int kIters = 150;
        for (const auto &kv : tiles) {
            const std::vector<int> &idxs = kv.second;
            if (idxs.size() < 500) {
                continue;
            }
            for (int round = 0; round < 2; ++round) {  // 每瓦片最多2个平面（如地面+墙裙）
                std::vector<int> avail;
                avail.reserve(idxs.size());
                for (int i : idxs) {
                    bool used_flag = false;
                    for (const auto &pl : protect_planes) {
                        if (std::fabs(pl.head<3>().dot(pts[i]) + pl.w()) < rayclean_plane_ransac_dist_ * 1.5) {
                            used_flag = true;
                            break;
                        }
                    }
                    if (!used_flag) {
                        avail.push_back(i);
                    }
                }
                if (avail.size() < std::max<size_t>(300, idxs.size() * rayclean_plane_min_inlier_frac_)) {
                    break;
                }
                int best_inliers = 0;
                Eigen::Vector4d best_plane(0, 0, 1, 0);
                std::mt19937 rng(static_cast<unsigned>(kv.first) + 20260830u + round * 7919u);
                std::uniform_int_distribution<int> uni(0, static_cast<int>(avail.size()) - 1);
                for (int it = 0; it < kIters; ++it) {
                    const Eigen::Vector3d &pa = pts[avail[uni(rng)]], &pb = pts[avail[uni(rng)]],
                                          &pc = pts[avail[uni(rng)]];
                    if ((pa - pb).norm() < 1.0 || (pa - pc).norm() < 1.0 || (pb - pc).norm() < 1.0) {
                        continue;
                    }
                    Eigen::Vector4d pl;
                    pl.head<3>() = (pb - pa).cross(pc - pa);
                    const double nrm = pl.head<3>().norm();
                    if (nrm < 1e-6) {
                        continue;
                    }
                    pl.head<3>() /= nrm;
                    pl.w() = -pl.head<3>().dot(pa);
                    int cnt = 0;
                    for (int i : avail) {
                        if (std::fabs(pl.head<3>().dot(pts[i]) + pl.w()) < rayclean_plane_ransac_dist_) {
                            cnt++;
                        }
                    }
                    if (cnt > best_inliers) {
                        best_inliers = cnt;
                        best_plane = pl;
                    }
                }
                if (best_inliers < static_cast<int>(std::max<size_t>(300, avail.size() * rayclean_plane_min_inlier_frac_))) {
                    break;
                }
                // 内点PCA精修
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                int m = 0;
                for (int i : avail) {
                    if (std::fabs(best_plane.head<3>().dot(pts[i]) + best_plane.w()) < rayclean_plane_ransac_dist_) {
                        mean += pts[i];
                        m++;
                    }
                }
                if (m > 3) {
                    mean /= m;
                    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                    for (int i : avail) {
                        if (std::fabs(best_plane.head<3>().dot(pts[i]) + best_plane.w()) < rayclean_plane_ransac_dist_) {
                            const Eigen::Vector3d ev = pts[i] - mean;
                            cov += ev * ev.transpose();
                        }
                    }
                    cov /= m;
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                    best_plane.head<3>() = es.eigenvectors().col(0);
                    best_plane.w() = -best_plane.head<3>().dot(mean);
                }
                protect_planes.push_back(best_plane);
                LOG(INFO) << "ray clean plane[" << protect_planes.size() - 1 << "]: inliers=" << best_inliers << "/"
                          << avail.size() << " n=[" << best_plane[0] << ", " << best_plane[1] << ", "
                          << best_plane[2] << "] d=" << best_plane[3];
            }
        }
        LOG(INFO) << "ray clean protect planes total: " << protect_planes.size();
    }
    // 体素保护掩码：距任一检测平面小于 protect_dist 的体素不参与清洗
    std::vector<uint8_t> protected_mask(n_cells, 0);
    for (int64_t i = 0; i < n_cells; ++i) {
        const int64_t ix = i / (ny * nz), iy = (i / nz) % ny, iz = i % nz;
        const Vec3d ctr(grid_min.x() + (ix + 0.5) * voxel_used, grid_min.y() + (iy + 0.5) * voxel_used,
                        grid_min.z() + (iz + 0.5) * voxel_used);
        for (const auto &pl : protect_planes) {
            if (std::fabs(pl.head<3>().dot(ctr) + pl.w()) < rayclean_plane_protect_dist_) {
                protected_mask[i] = 1;
                break;
            }
        }
    }
    // near_protected: 自身或26邻域存在保护体素（贴近平面带）。迟到穿越判据只在远离
    // 平面带的位置生效——漂移窗格（表面观测提前终止的静态体素）必然贴着平面带，
    // 而多拨行人反复经过的悬空残影远离平面带，以此门控区分二者
    std::vector<uint8_t> near_protected(n_cells, 0);
    for (int64_t i = 0; i < n_cells; ++i) {
        if (!protected_mask[i]) {
            continue;
        }
        const int64_t ix = i / (ny * nz), iy = (i / nz) % ny, iz = i % nz;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    const int64_t jx = ix + dx, jy = iy + dy, jz = iz + dz;
                    if (jx < 0 || jx >= nx || jy < 0 || jy >= ny || jz < 0 || jz >= nz) {
                        continue;
                    }
                    near_protected[(jx * ny + jy) * nz + jz] = 1;
                }
    }
    // 迟到穿越计数：最后一次被观测为表面之后，仍有多少个不同关键帧的射线穿过该体素
    std::vector<std::atomic<uint16_t>> late_cnt(n_cells);
    for (int64_t i = 0; i < n_cells; ++i) {
        late_cnt[i].store(0);
    }

    // 无锁更新体素端点计数（同一关键帧只记一次；不同线程处理不同关键帧，CAS安全）
    auto add_end = [&](int64_t cell_idx, int kf_idx) {
        uint64_t cur = cells[cell_idx].load(std::memory_order_relaxed);
        while (true) {
            uint64_t end_cnt = (cur >> 32) & 0xFFFF;
            uint64_t last_end = cur & 0xFFFF;
            if (last_end == (uint64_t)kf_idx) {
                return;  // 本关键帧已计入
            }
            uint64_t next = (cur & 0xFFFFFFFF0000FFFF) | (((end_cnt + 1) & 0xFFFF) << 32) | (uint64_t)kf_idx;
            if (cells[cell_idx].compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
                return;
            }
        }
    };

    // 无锁更新体素穿越计数（同一关键帧只记一次）
    auto add_pass = [&](int64_t cell_idx, int kf_idx) {
        uint64_t cur = cells[cell_idx].load(std::memory_order_relaxed);
        while (true) {
            uint64_t pass_cnt = (cur >> 48) & 0xFFFF;
            uint64_t last_pass = (cur >> 16) & 0xFFFF;
            if (last_pass == (uint64_t)kf_idx) {
                return;
            }
            uint64_t next = (cur & 0x0000FFFFFFFF0000) | (((pass_cnt + 1) & 0xFFFF) << 48) | ((uint64_t)kf_idx << 16) |
                            (cur & 0xFFFF);
            if (cells[cell_idx].compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
                return;
            }
        }
    };

    const unsigned n_thread = std::min(4u, std::max(1u, std::thread::hardware_concurrency() / 2));

    /// ---- 第1遍：端点计数 + 每体素点数 ----
    {
        std::vector<std::thread> workers;
        std::atomic<int> next_kf(0);
        for (unsigned t = 0; t < n_thread; ++t) {
            workers.emplace_back([&]() {
                int i;
                while ((i = next_kf.fetch_add(1)) < n_kf) {
                    const auto &cloud = kfs[i]->GetCloud();
                    if (!cloud) {
                        continue;
                    }
                    for (const auto &pt : cloud->points) {
                        Vec3d pw = kf_rot[i] * pt.getVector3fMap().cast<double>() + kf_pos[i];
                        if ((pw - kf_pos[i]).squaredNorm() > max_range_sq) {
                            continue;
                        }
                        int64_t ix, iy, iz;
                        if (!vox_index(pw, ix, iy, iz)) {
                            continue;
                        }
                        int64_t idx = (ix * ny + iy) * nz + iz;
                        add_end(idx, i);
                        uint16_t c = pt_cnt[idx].load(std::memory_order_relaxed);
                        while (c < 0xFFFF && !pt_cnt[idx].compare_exchange_weak(c, c + 1, std::memory_order_relaxed)) {
                        }
                    }
                }
            });
        }
        for (auto &w : workers) {
            w.join();
        }
    }

    /// ---- 第2遍：射线穿越计数（DDA，每stride个点取一条射线）----
    {
        std::vector<std::thread> workers;
        std::atomic<int> next_kf(0);
        for (unsigned t = 0; t < n_thread; ++t) {
            workers.emplace_back([&]() {
                std::vector<int32_t> last_seen_late(n_cells, -1);
                int i;
                while ((i = next_kf.fetch_add(1)) < n_kf) {
                    const auto &cloud = kfs[i]->GetCloud();
                    if (!cloud) {
                        continue;
                    }
                    const Vec3d &o = kf_pos[i];
                    const int stride = rayclean_ray_stride_;
                    for (size_t pi = 0; pi < cloud->points.size(); pi += stride) {
                        const auto &pt = cloud->points[pi];
                        Vec3d e = kf_rot[i] * pt.getVector3fMap().cast<double>() + kf_pos[i];
                        Vec3d d = e - o;
                        double ray_len = d.norm();
                        if (ray_len < min_range || ray_len > rayclean_max_range_) {
                            continue;
                        }
                        d /= ray_len;

                        int64_t ix, iy, iz;
                        if (!vox_index(o, ix, iy, iz)) {
                            continue;  // 原点应在栅格内（建包围盒时已包含）
                        }
                        const int step_x = d.x() > 0 ? 1 : -1;
                        const int step_y = d.y() > 0 ? 1 : -1;
                        const int step_z = d.z() > 0 ? 1 : -1;
                        // 到达下一体素边界的参数t（沿单位方向）
                        const double t_delta_x = d.x() != 0 ? voxel_used / std::fabs(d.x()) : 1e18;
                        const double t_delta_y = d.y() != 0 ? voxel_used / std::fabs(d.y()) : 1e18;
                        const double t_delta_z = d.z() != 0 ? voxel_used / std::fabs(d.z()) : 1e18;
                        double t_max_x =
                            (d.x() != 0) ? ((ix + (step_x > 0 ? 1 : 0)) * voxel_used + grid_min.x() - o.x()) / d.x() : 1e18;
                        double t_max_y =
                            (d.y() != 0) ? ((iy + (step_y > 0 ? 1 : 0)) * voxel_used + grid_min.y() - o.y()) / d.y() : 1e18;
                        double t_max_z =
                            (d.z() != 0) ? ((iz + (step_z > 0 ? 1 : 0)) * voxel_used + grid_min.z() - o.z()) / d.z() : 1e18;

                        const int max_steps = static_cast<int>(3.0 * ray_len * inv_voxel) + 8;
                        for (int s = 0; s < max_steps; ++s) {
                            double t_next;
                            if (t_max_x <= t_max_y && t_max_x <= t_max_z) {
                                t_next = t_max_x;
                                t_max_x += t_delta_x;
                                ix += step_x;
                            } else if (t_max_y <= t_max_z) {
                                t_next = t_max_y;
                                t_max_y += t_delta_y;
                                iy += step_y;
                            } else {
                                t_next = t_max_z;
                                t_max_z += t_delta_z;
                                iz += step_z;
                            }
                            if (t_next > ray_len) {
                                break;  // 射线在当前体素内结束
                            }
                            if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz) {
                                break;  // 出栅格
                            }
                            // 距终点过近的体素不计穿越：保护静态表面自身及其贴邻体素
                            Vec3d center(grid_min.x() + (ix + 0.5) * voxel_used, grid_min.y() + (iy + 0.5) * voxel_used,
                                         grid_min.z() + (iz + 0.5) * voxel_used);
                            if ((center - e).squaredNorm() < guard_dist_sq) {
                                continue;
                            }
                            const int64_t pidx = (ix * ny + iy) * nz + iz;
                            add_pass(pidx, i);
                            // 迟到穿越：本次穿越发生在该体素最后一次被观测为表面之后
                            // （last_end 从 pass-1 写入的 cell 字低位段读取）
                            if (last_seen_late[pidx] != i) {
                                const uint64_t cur = cells[pidx].load(std::memory_order_relaxed);
                                const uint32_t last_end = static_cast<uint32_t>(cur & 0xFFFF);
                                if (last_end != 0 && i > static_cast<int>(last_end)) {
                                    last_seen_late[pidx] = i;
                                    uint16_t lc = late_cnt[pidx].load(std::memory_order_relaxed);
                                    while (lc < 0xFFFF && !late_cnt[pidx].compare_exchange_weak(
                                                             lc, lc + 1, std::memory_order_relaxed)) {
                                    }
                                }
                            }
                        }
                    }
                }
            });
        }
        for (auto &w : workers) {
            w.join();
        }
    }

    /// ---- 第3遍：判决并改写各关键帧点云 ----
    // 动态体素：被>=pass_th个不同关键帧的射线穿过（说明该处应为自由空间），
    //           且仅有<=end_th个不同关键帧观测到表面（静态表面会被反复观测，计数高）
    size_t removed_by_ray = 0, removed_by_iso = 0, kept_pts = 0, restored_sparse = 0;
    if (rayclean_debug_save_) {
        ray_removed_dbg_cloud_.reset(new PointCloudType());
        ray_removed_dbg_cloud_->points.reserve(200000);
    }

    /// ---- 第3.5遍：射线判决候选的空间密度统计 ----
    // 只计数满足动态残影判据的"候选点"；真正删除前要求其体素+26邻域内候选总数达标。
    // 行人残影点集稠密、轻易达到阈值；墙面/天花板被掠射零星误判的孤立候选四周无同伙，放回
    auto ray_condemn_check = [&](int64_t idx) -> bool {
        const uint64_t c = cells[idx].load(std::memory_order_relaxed);
        const uint64_t pass_cnt = (c >> 48) & 0xFFFF;
        const uint64_t end_cnt = (c >> 32) & 0xFFFF;
        // 原判据：被穿越 且 观测次数少（快速通过的行人）
        if (pass_cnt >= (uint64_t)rayclean_pass_th_ && end_cnt >= 1 && end_cnt <= (uint64_t)rayclean_end_th_) {
            return true;
        }
        // 迟到穿越判据：最后一次被观测为表面之后仍被>=late_pass_th个不同关键帧穿越，
        // 且远离平面保护带（多拨行人反复经过/久站后离开的悬空残影；贴平面带的漂移窗格被门控排除）
        if (!near_protected[idx] && late_cnt[idx].load(std::memory_order_relaxed) >= (uint16_t)rayclean_late_pass_th_) {
            return true;
        }
        return false;
    };
    std::vector<std::atomic<uint16_t>> cand_cnt(n_cells);
    for (int64_t i = 0; i < n_cells; ++i) {
        cand_cnt[i].store(0);
    }
    for (int i = 0; i < n_kf; ++i) {
        const auto &cloud = kfs[i]->GetCloud();
        if (!cloud) {
            continue;
        }
        for (const auto &pt : cloud->points) {
            Vec3d pw = kf_rot[i] * pt.getVector3fMap().cast<double>() + kf_pos[i];
            if ((pw - kf_pos[i]).squaredNorm() > max_range_sq) {
                continue;
            }
            int64_t ix, iy, iz;
            if (!vox_index(pw, ix, iy, iz)) {
                continue;
            }
            int64_t idx = (ix * ny + iy) * nz + iz;
            if (protected_mask[idx]) {
                continue;  // 平面保护：地面/墙面等结构性平面不参与清洗
            }
            if (ray_condemn_check(idx)) {
                uint16_t c = cand_cnt[idx].load(std::memory_order_relaxed);
                while (c < 0xFFFF && !cand_cnt[idx].compare_exchange_weak(c, c + 1, std::memory_order_relaxed)) {
                }
            }
        }
    }

    /// ---- 第3.6遍：邻域生长吸收残影边缘 ----
    // 已确认删除的残影核心周围，常残留同一动态物体的边缘点（观测次数略高或聚类支持不足）。
    // 从确认体素出发BFS生长，吸收连通的、有动态证据（被穿越 且 观测次数<=grow_end_th）且非
    // 保护区、含点的邻域体素；地面/墙面等静态表面被平面保护且观测次数远超阈值，生长自然
    // 止步于残影边界
    std::vector<uint8_t> remove_mask(n_cells, 0);
    size_t grown_cells = 0;
    if (rayclean_grow_en_) {
        // 种子：判据通过 且 聚类支持达标（与第3遍逐点判定一致，按体素预计算）
        std::vector<int64_t> frontier;
        for (int64_t i = 0; i < n_cells; ++i) {
            if (protected_mask[i]) {
                continue;
            }
            if (!ray_condemn_check(i)) {
                continue;
            }
            const int64_t ix = i / (ny * nz), iy = (i / nz) % ny, iz = i % nz;
            int support = 0;
            for (int64_t dx = -1; dx <= 1 && support < rayclean_min_cluster_points_; ++dx)
                for (int64_t dy = -1; dy <= 1 && support < rayclean_min_cluster_points_; ++dy)
                    for (int64_t dz = -1; dz <= 1 && support < rayclean_min_cluster_points_; ++dz) {
                        const int64_t jx = ix + dx, jy = iy + dy, jz = iz + dz;
                        if (jx < 0 || jx >= nx || jy < 0 || jy >= ny || jz < 0 || jz >= nz) {
                            continue;
                        }
                        support += cand_cnt[(jx * ny + jy) * nz + jz].load(std::memory_order_relaxed);
                    }
            if (support >= rayclean_min_cluster_points_) {
                remove_mask[i] = 1;  // 1=确认种子
                frontier.push_back(i);
            }
        }
        // BFS 生长
        for (int step = 0; step < rayclean_grow_steps_ && !frontier.empty(); ++step) {
            std::vector<int64_t> next;
            for (const int64_t i : frontier) {
                const int64_t ix = i / (ny * nz), iy = (i / nz) % ny, iz = i % nz;
                for (int d = 0; d < 27; ++d) {
                    const int64_t jx = ix + d / 9 - 1, jy = iy + (d / 3) % 3 - 1, jz = iz + d % 3 - 1;
                    if (jx < 0 || jx >= nx || jy < 0 || jy >= ny || jz < 0 || jz >= nz) {
                        continue;
                    }
                    const int64_t j = (jx * ny + jy) * nz + jz;
                    if (remove_mask[j] || protected_mask[j]) {
                        continue;
                    }
                    if (pt_cnt[j].load(std::memory_order_relaxed) == 0) {
                        continue;  // 只吸收含点的体素
                    }
                    const uint64_t c = cells[j].load(std::memory_order_relaxed);
                    const uint64_t pass_cnt = (c >> 48) & 0xFFFF;
                    const uint64_t end_cnt = (c >> 32) & 0xFFFF;
                    const bool late_ok = !near_protected[j] &&
                                         late_cnt[j].load(std::memory_order_relaxed) >=
                                             (uint16_t)rayclean_late_pass_th_;
                    if (pass_cnt < (uint64_t)rayclean_pass_th_ || end_cnt < 1 ||
                        (!(late_ok || end_cnt <= (uint64_t)rayclean_grow_end_th_))) {
                        continue;  // 无穿越证据，或观测次数过高且无迟到穿越（静态表面），不吸收
                    }
                    remove_mask[j] = 2;  // 2=邻域生长吸收
                    grown_cells++;
                    next.push_back(j);
                }
            }
            frontier = std::move(next);
        }
        LOG(INFO) << "ray clean grow: absorbed_cells=" << grown_cells << " steps=" << rayclean_grow_steps_;
    }

    for (int i = 0; i < n_kf; ++i) {
        const auto &cloud = kfs[i]->GetCloud();
        if (!cloud) {
            continue;
        }
        CloudPtr cleaned(new PointCloudType());
        cleaned->points.reserve(cloud->points.size());

        for (const auto &pt : cloud->points) {
            Vec3d pw = kf_rot[i] * pt.getVector3fMap().cast<double>() + kf_pos[i];
            bool in_grid = (pw - kf_pos[i]).squaredNorm() <= max_range_sq;
            int64_t ix = 0, iy = 0, iz = 0;
            if (in_grid) {
                in_grid = vox_index(pw, ix, iy, iz);
            }

            bool remove = false;
            bool by_ray = false;
            if (in_grid) {
                int64_t idx = (ix * ny + iy) * nz + iz;
                if (protected_mask[idx]) {
                    in_grid = false;  // 平面保护：地面/墙面点一律保留
                }
            }
            if (in_grid) {
                int64_t idx = (ix * ny + iy) * nz + iz;
                const uint8_t mk = remove_mask[idx];
                if (mk != 0) {
                    remove = true;  // 动态残影（种子确认或邻域生长吸收）
                    by_ray = true;
                } else if (ray_condemn_check(idx)) {
                    restored_sparse++;  // 候选但未达聚类支持，零星放回
                } else if (rayclean_iso_remove_ && pt_cnt[idx].load(std::memory_order_relaxed) == 1) {
                    // 孤立噪点：自身体素只有1个点且26邻域也全空（天花板噪点等）
                    bool has_nb = false;
                    for (int64_t dx = -1; dx <= 1 && !has_nb; ++dx)
                        for (int64_t dy = -1; dy <= 1 && !has_nb; ++dy)
                            for (int64_t dz = -1; dz <= 1 && !has_nb; ++dz) {
                                if (dx == 0 && dy == 0 && dz == 0) {
                                    continue;
                                }
                                int64_t jx = ix + dx, jy = iy + dy, jz = iz + dz;
                                if (jx < 0 || jx >= nx || jy < 0 || jy >= ny || jz < 0 || jz >= nz) {
                                    continue;
                                }
                                if (pt_cnt[(jx * ny + jy) * nz + jz].load(std::memory_order_relaxed) > 0) {
                                    has_nb = true;
                                }
                            }
                    if (!has_nb) {
                        remove = true;  // 孤立噪点
                    }
                }
            }

            if (!remove) {
                cleaned->points.emplace_back(pt);
                kept_pts++;
            } else {
                if (by_ray) {
                    removed_by_ray++;
                } else {
                    removed_by_iso++;
                }
                if (rayclean_debug_save_) {
                    PointType dp;
                    dp.x = pw.x();
                    dp.y = pw.y();
                    dp.z = pw.z();
                    dp.intensity = pt.intensity;
                    dp.time = 0;
                    ray_removed_dbg_cloud_->points.emplace_back(dp);
                }
            }
        }
        cleaned->width = static_cast<uint32_t>(cleaned->points.size());
        cleaned->height = 1;
        cleaned->is_dense = false;
        kfs[i]->SetCloud(cleaned);
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double cost_ms = std::chrono::duration<double, std::milli>(t_end - t_begin).count();
    LOG(INFO) << "ray clean: kfs=" << n_kf << " points=" << total_pts << " kept=" << kept_pts
              << " removed_by_ray=" << removed_by_ray << " removed_by_iso=" << removed_by_iso
              << " restored_sparse=" << restored_sparse << " cost=" << cost_ms << "ms voxel=" << voxel_used
              << " grid=" << (long long)nx << "x" << (long long)ny << "x" << (long long)nz << " threads=" << n_thread;
}

void LaserMapping::SaveRayRemovedCloud(const std::string &dir) {
    if (!rayclean_debug_save_) {
        LOG(INFO) << "ray removed points not saved: rayclean_debug_save is disabled";
        return;
    }

    if (ray_removed_dbg_cloud_ == nullptr || ray_removed_dbg_cloud_->points.empty()) {
        LOG(INFO) << "ray removed points not saved: no points removed";
        return;
    }

    std::string path = dir.empty() ? "./data/ray_removed_pts.pcd" : dir + "/ray_removed_pts.pcd";
    if (pcl::io::savePCDFileBinary(path, *ray_removed_dbg_cloud_) != 0) {
        LOG(ERROR) << "failed to save ray removed points to " << path;
        return;
    }

    LOG(INFO) << "ray removed points saved to " << path << ", points: " << ray_removed_dbg_cloud_->points.size();
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

}  // namespace lightning
