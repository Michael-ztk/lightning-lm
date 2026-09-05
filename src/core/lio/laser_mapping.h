#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"
#include "pointcloud_preprocess.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

namespace ui {
class PangolinWindow;
}

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        /// 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    /// init without ros
    bool Init(const std::string &config_yaml);

    bool Run();

    // callbacks of lidar and imu
    /// 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    /// 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    /// 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    /// 轮速里程计数据入口（body系线速度），用于退化场景的速度观测约束
    void ProcessOdom(const OdomPtr &odomData);

    /// 保存前端的地图
    void SaveMap();

    /**
     * @brief 射线free-space清洗：建图结束、位姿最终确定后调用（保存地图前）
     *
     * 用所有关键帧的视线证据清除各关键帧点云中的动态残影（行人等动态物体走过留下的点）：
     * 行人曾在的体素，其走开后会被后续关键帧的射线反复穿过（射线终点在其后的静态表面上），
     * "被多个不同关键帧穿过 + 很少被作为表面观测"即判为动态体素，删除其中全部点。
     * 直接改写关键帧点云，之后导出的点云图与栅格图同时变干净。离线一次执行，不占运行时CPU。
     */
    void RemoveDynamicByKeyframeRays();

    /// 保存被射线清洗删除的点(世界系)到 dir/ray_removed_pts.pcd，用于误删检查
    void SaveRayRemovedCloud(const std::string &dir);

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    void SetPose(const SE3& pose) { kf_.SetPose(pose); }

    /// 获取关键帧
    Keyframe::Ptr GetKeyframe() const { return last_kf_; }

    /// 获取激光的状态
    NavState GetState() const { return state_point_; }

    /// 获取IMU状态
    NavState GetIMUState() const {
        if (p_imu_->IsIMUInited()) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    CloudPtr GetScanUndist() const { return scan_undistort_; }

    /// 获取最新的点云
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() {
        UL lock(mtx_keyframes_);
        return all_keyframes_;
    }

    /**
     * 计算全局地图
     * @param use_lio_pose
     * @return
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

   private:
    // sync lidar with imu
    bool SyncPackages();

    void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    /// 轮速速度观测模型（序贯紧耦合，退化方向由轮速约束）
    void WheelSpeedModel(NavState &s, ESKF::CustomObservationModel &obs);

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot_ * (state_point_.offset_R_lidar_ * pi.getVector3fMap().cast<double>() +
                                            state_point_.offset_t_lidar_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    bool LoadParamsFromYAML(const std::string &yaml);

    /// 创建关键帧
    void MakeKF();

   private:
    Options options_;

    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    std::string map_file_path_;

    /// 射线free-space清洗参数（保存地图前离线执行）
    bool rayclean_en_ = true;              // 是否开启射线清洗
    double rayclean_voxel_size_ = 0.2;     // 体素边长(m)
    int rayclean_pass_th_ = 2;             // 体素被>=该数量的不同关键帧射线穿过（且表面观测少）判为动态残影
    int rayclean_end_th_ = 3;              // 体素内出现<=该数量的不同关键帧端点，配合穿越数判动态
    double rayclean_plane_protect_dist_ = 0.3;   // 平面保护：体素中心距检测平面小于该值(m)则不参与清洗
    double rayclean_plane_tile_size_ = 10.0;     // 分块RANSAC的xy瓦片边长(m)
    double rayclean_plane_ransac_dist_ = 0.15;   // RANSAC平面内点距离阈值(m)
    double rayclean_plane_min_inlier_frac_ = 0.05;  // 平面最小内点比例(占瓦片点数)
    bool rayclean_grow_en_ = true;         // 邻域生长：从已确认残影核心出发，吸收连通的动态边缘残余
    int rayclean_grow_end_th_ = 12;        // 生长吸收判据：邻域体素端点观测次数<=该值（静态表面观测次数远高于此，生长自然止步）
    int rayclean_grow_steps_ = 12;         // 生长最大步数（体素）
    int rayclean_late_pass_th_ = 2;        // 迟到穿越判据：体素在最后一次被观测为表面之后仍被>=该数量的不同关键帧
                                           // 穿越，且邻域无保护体素（远离地面/墙面的悬空位置），判为动态残影。
                                           // 用于找回"多拨行人反复经过同一区域"导致的观测次数累计超标的残影；
                                           // 漂移窗格紧贴保护平面带，被 near_protected 门控排除，不会误伤结构
    int rayclean_min_cluster_points_ = 20; // 射线判决候选的邻域最少点数，低于此数的零星候选放回（防墙面/天花板零星误删）
    double rayclean_max_range_ = 20.0;     // 只统计该距离(m)内的点与射线
    int rayclean_ray_stride_ = 2;          // 射线采样步长，每stride个点取一条射线，控制计算量
    double rayclean_guard_dist_ = 0.3;     // 距射线终点小于该距离(m)的体素不计穿越，保护表面贴邻体素
    bool rayclean_iso_remove_ = false;     // 顺带删除孤立噪点（自身体素及26邻域内无其他点）
    bool rayclean_debug_save_ = true;      // 调试：保存被射线清洗删除的点(世界系)到pcd
    CloudPtr ray_removed_dbg_cloud_ = nullptr;  // 累积的被射线清洗删除的点(世界系)

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;
    std::mutex mtx_keyframes_;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    std::vector<Vec4f> corr_pts_;                     // inlier pts
    std::vector<Vec4f> corr_norm_;                    // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    std::vector<float> residuals_;           // point-to-plane residuals
    std::vector<char> point_selected_surf_;  // selected points
    std::vector<Vec4f> plane_coef_;          // plane coeffs

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;
    std::deque<OdomPtr> odom_buffer_;  // 轮速里程计缓冲（与雷达/IMU共用mtx_buffer_）

    /// options
    bool keep_first_imu_estimation_ = false;    // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_odom_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    /// 轮速融合参数与当帧观测
    bool odom_en_ = false;                 // 是否启用轮速融合
    double odom_vel_noise_ = 0.01;         // 轮速线速度观测噪声 (m/s)^2，σ=0.1m/s
    double odom_pos_noise_floor_ = 0.002;  // 位移观测噪声下限(m)，仅数值兜底，应远小于基础噪声σ_vel·dt
    double odom_pos_noise_ratio_ = 0.05;   // 位移观测的比例噪声(无量纲)，模型化轮径误差/打滑
    double odom_max_time_diff_ = 0.2;      // 轮速与帧尾时间最大允许偏差(s)
    double odom_yaw_offset_ = 0.0;         // 底盘系相对IMU系的yaw偏移(rad)
    Vec3d cur_wheel_vel_ = Vec3d::Zero();               // 当帧轮速观测（body系）
    Vec3d fwd_body_ = Vec3d(1.0, 0.0, 0.0);             // body系车头方向单位向量，由 odom_yaw_offset 派生
    bool has_wheel_obs_ = false;                        // 当帧是否有可用轮速观测

    // 位移增量观测：记录上一帧融合后位姿，用 (p_cur-p_prev) 与 R·v_body·dt 的残差约束位置，
    // 避免雷达在退化方向钉死绝对位置时轮速因只观测速度而推不动位姿。
    // 注意：p_prev 记录的是上一帧做完 LIDAR+WHEEL 更新后的位置，其时间基准是上一帧的
    // lidar_end_time_，因此 wheel_obs_dt_ 也必须用帧尾时间差，不能再用 odom 时间戳差。
    Vec3d last_wheel_pos_ = Vec3d::Zero();  // 上一帧融合后的世界系位置
    double last_wheel_time_ = -1.0;         // 上一帧帧尾时间(s)，与 last_wheel_pos_ 同基准
    double wheel_obs_dt_ = 0.0;             // 当帧与上帧的帧尾时间差(s)
    bool has_last_wheel_ = false;           // 是否已有上一帧轮速位姿（首帧无增量）

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_num_ = 0, frame_num_ = 0;

    double last_lidar_time_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    Vec3d pos_lidar_;  // lidar position after eskf update
    SO3 euler_cur_;    // rotation in euler angles
    bool extrinsic_est_en_ = true;
    bool use_aa_ = false;  // use anderson acceleration?

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H
