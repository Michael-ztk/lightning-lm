#ifndef LIGHTNING_SYSTEM_H
#define LIGHTNING_SYSTEM_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "lightning/srv/system_command.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

class LaserMapping;
class LoopClosing;

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

namespace loc {
class Localization;
}

class LightningSystem {
   public:
    enum class Mode { IDLE, MAPPING, LOCALIZING };

    LightningSystem();
    ~LightningSystem();

    bool Init(const std::string& yaml_path);
    void Spin();

   private:
    void StopCurrentMode();
    bool StartMappingInternal(const std::string& map_name);
    bool StartLocalizationInternal(const std::string& map_path);

    // Service 回调
    void OnCommand(const srv::SystemCommand::Request::SharedPtr req, srv::SystemCommand::Response::SharedPtr res);

    // 传感器回调
    void ProcessIMU(const IMUPtr& imu);
    void ProcessLidarPC2(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidarLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    // 建图模式
    void PostLidarRunMapping();
    void SaveMapInternal(const std::string& save_path);
    void PublishTF(const SE3& pose);
    void UpdateVisualizationCaches();
    void PublishVisualizationLoop();

    // 定位模式
    void OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg);
    void PublishStaticPCD(const std::string& pcd_path);

    // 状态
    std::atomic<Mode> current_mode_{Mode::IDLE};
    std::atomic_bool running_{false};
    std::string yaml_path_;
    std::string current_map_name_;

    // ROS2 基础设施
    rclcpp::Node::SharedPtr node_;
    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;
    bool imu_in_g_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Service<srv::SystemCommand>::SharedPtr command_service_;

    // 建图模式组件
    std::shared_ptr<LaserMapping> lio_;
    std::shared_ptr<LoopClosing> lc_;
    std::shared_ptr<g2p5::G2P5> g2p5_;
    std::shared_ptr<ui::PangolinWindow> ui_;
    Keyframe::Ptr cur_kf_;

    bool with_loop_closing_ = true;
    bool with_visualization_ = true;
    bool with_2dvisualization_ = true;
    bool with_gridmap_ = true;
    bool step_on_kf_ = false;

    double map_publish_interval_ = 1.0;
    const size_t max_viz_points_ = 1000000;
    const float viz_voxel_size_ = 0.2f;

    std::thread viz_publish_thread_;
    std::atomic_bool stop_viz_publish_thread_{false};
    std::atomic_bool force_global_map_publish_{false};
    std::mutex viz_publish_mutex_;
    CloudPtr latest_registered_scan_;
    SE3 latest_registered_scan_pose_;
    uint64_t latest_registered_scan_seq_ = 0;
    CloudPtr global_map_cache_{new PointCloudType()};
    uint64_t global_map_cache_seq_ = 0;
    bool global_map_needs_full_replace_ = false;
    std::atomic_ulong latest_kf_id_for_viz_{0};
    std::atomic_bool has_kf_for_viz_{false};

    std::thread save_thread_;
    std::atomic_bool saving_map_{false};

    // 定位模式组件
    std::shared_ptr<loc::Localization> loc_;
    std::atomic_bool loc_started_{false};
};

}  // namespace lightning

#endif  // LIGHTNING_SYSTEM_H
