//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");
    imu_in_g_ = yaml.GetValue<bool>("common", "imu_in_g");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            if (imu_in_g_) {
                imu->linear_acceleration *= 9.81;
            }

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", false);
        });

    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
            Eigen::Vector3d position(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                                     pose_msg->pose.pose.position.z);

            Eigen::Quaterniond quaternion(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
                                          pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);

            SE3 init_pose(quaternion, position);

            LOG(INFO) << "Received initial pose from rviz2: pos=" << position.transpose()
                      << ", quat=" << quaternion.coeffs().transpose();

            SetInitPose(init_pose);
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) { tf_broadcaster_->sendTransform(pose); });
    }

    if (options_.pub_static_pcd_) {
        rclcpp::QoS latching_qos(1);
        latching_qos.transient_local();
        static_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/static_map", latching_qos);
    }

    registered_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 1);
    loc_->SetRegisteredScanCallback([this](const sensor_msgs::msg::PointCloud2& registered_scan) {
        if (registered_scan_pub_ && !registered_scan.data.empty()) {
            registered_scan_pub_->publish(registered_scan);
        }
    });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";

        if (options_.pub_static_pcd_ && static_map_pub_) {
            PublishStaticPCD();
        }
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::PublishStaticPCD() {
    try {
        CloudPtr global_map(new PointCloudType);
        if (pcl::io::loadPCDFile<PointType>(options_.global_pcd_path_, *global_map) == -1) {
            LOG(ERROR) << "Failed to load global PCD file: " << options_.global_pcd_path_;
            return;
        }

        if (global_map->empty()) {
            LOG(WARNING) << "Global PCD file is empty: " << options_.global_pcd_path_;
            return;
        }

        CloudPtr filtered_map = math::VoxelGrid(global_map, 0.2);

        if (filtered_map->empty()) {
            LOG(WARNING) << "Filtered static PCD map is empty: " << options_.global_pcd_path_;
            return;
        }

        sensor_msgs::msg::PointCloud2 static_map_msg;
        pcl::toROSMsg(*filtered_map, static_map_msg);
        static_map_msg.header.frame_id = "map";
        static_map_msg.header.stamp = node_->now();

        static_map_pub_->publish(static_map_msg);

        LOG(INFO) << "Published filtered static PCD map with " << filtered_map->size() << "/" << global_map->size()
                  << " points from: " << options_.global_pcd_path_;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception while publishing static PCD: " << e.what();
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
