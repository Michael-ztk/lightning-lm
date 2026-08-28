//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
        lc_->SetLoopClosedCB([this]() {
            if (global_map_pub_) {
                auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_, true, 0.3);
                if (global_map) {
                    UL lock(viz_publish_mutex_);
                    global_map_cache_ = global_map;
                    global_map_cache_seq_++;
                    global_map_needs_full_replace_ = true;
                }
                force_global_map_publish_.store(true);
            }

            if (g2p5_) {
                g2p5_->RedrawGlobalMap();
            }
        });
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
        imu_in_g_ = yaml["common"]["imu_in_g"].as<bool>();

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                if (imu_in_g_) {
                    imu->linear_acceleration *= 9.81;
                }

                static double last_imu_header_time = -1.0;
                static auto last_imu_callback_wall_time = std::chrono::steady_clock::time_point{};
                const auto now_wall_time = std::chrono::steady_clock::now();
                const double header_dt = last_imu_header_time > 0 ? (imu->timestamp - last_imu_header_time) : 0.0;
                const double callback_wall_dt =
                    last_imu_callback_wall_time != std::chrono::steady_clock::time_point{}
                        ? std::chrono::duration<double>(now_wall_time - last_imu_callback_wall_time).count()
                        : 0.0;

                if (last_imu_header_time > 0 && (header_dt > 0.1 || callback_wall_dt > 0.1)) {
                    LOG(WARNING) << std::fixed << std::setprecision(9)
                                 << "[imu_callback] header_dt=" << header_dt
                                 << ", wall_dt=" << callback_wall_dt << ", stamp=" << imu->timestamp;
                }

                last_imu_header_time = imu->timestamp;
                last_imu_callback_wall_time = now_wall_time;

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", false);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", false);
            });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        // 发布地图点云到RViz
        if (yaml["system"]["publish_map_to_rviz"].as<bool>(false)) {
            map_publish_interval_ = yaml["system"]["map_publish_interval"].as<double>(1.0);
            global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", qos);
            LOG(INFO) << "global map publisher created, topic: lightning/global_map, interval: "
                      << map_publish_interval_ << "s";
        }

        // 发布TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

        // 发布对齐后的当前帧点云
        if (yaml["system"]["pub_registered_scan"].as<bool>(false)) {
            registered_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", qos);
        }

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    stop_viz_publish_thread_ = true;
    if (viz_publish_thread_.joinable()) {
        viz_publish_thread_.join();
    }

    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::WaitForUIExit() {
    if (!ui_) {
        return;
    }

    while (!ui_->ShouldQuit() && !debug::flg_exit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // 射线free-space清洗：位姿已最终确定（回环优化后），在导出前用全部关键帧的视线证据
    // 清除动态残影（行人等）与孤立噪点，直接改写关键帧点云，点云图与栅格图同时变干净
    lio_->RemoveDynamicByKeyframeRays();

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptLidarPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    // 保存被射线清洗过滤的动态点，用于误删检查
    lio_->SaveRayRemovedCloud(save_path);

    if (options_.with_gridmap_) {
        /// 使用优化后的位姿重建栅格地图
        g2p5_->RedrawGlobalMap();
        /// 等待后端线程检测到 flag 并开始重建
        while (!g2p5_->IsBusy()) {
            usleep(100000);
        }
        /// 等待重建完成
        while (g2p5_->IsBusy()) {
            usleep(100000);
        }

        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    auto state = lio_->GetState();
    if (state.pose_is_ok_) {
        PublishTF(state.GetPose());
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    UpdateVisualizationCaches();
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    auto state = lio_->GetState();
    if (state.pose_is_ok_) {
        PublishTF(state.GetPose());
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    UpdateVisualizationCaches();
}

void SlamSystem::PublishTF(const SE3& pose) {
    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node_->now();
        t.header.frame_id = "map";
        t.child_frame_id = "body";
        t.transform.translation.x = pose.translation()(0);
        t.transform.translation.y = pose.translation()(1);
        t.transform.translation.z = pose.translation()(2);
        auto q = pose.unit_quaternion();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }
}

void SlamSystem::UpdateVisualizationCaches() {
    latest_kf_id_for_viz_.store(cur_kf_->GetID());
    has_kf_for_viz_.store(true);

    const SE3 pose = options_.with_loop_closing_ ? cur_kf_->GetOptBodyPose() : cur_kf_->GetLIOBodyPose();

    if (registered_scan_pub_ && registered_scan_pub_->get_subscription_count() > 0) {
        auto scan = lio_->GetScanUndist();
        if (scan && scan->size() > 0) {
            CloudPtr scan_copy(new PointCloudType(*scan));
            {
                UL lock(viz_publish_mutex_);
                latest_registered_scan_ = scan_copy;
                latest_registered_scan_pose_ = pose;
                latest_registered_scan_seq_++;
            }
        }
    }

    if (global_map_pub_) {
        CloudPtr cloud = cur_kf_->GetCloud();
        if (cloud && cloud->size() > 0) {
            pcl::VoxelGrid<PointType> voxel;
            voxel.setLeafSize(1.0f, 1.0f, 1.0f);

            CloudPtr cloud_filter(new PointCloudType);
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, pose.matrix());
            cloud_trans->is_dense = false;
            cloud_trans->height = 1;
            cloud_trans->width = cloud_trans->size();

            UL lock(viz_publish_mutex_);
            *global_map_cache_ += *cloud_trans;
            global_map_cache_->is_dense = false;
            global_map_cache_->height = 1;
            global_map_cache_->width = global_map_cache_->size();
            global_map_cache_seq_++;
        }
    }
}

void SlamSystem::PublishVisualizationLoop() {
    rclcpp::Rate loop_rate(20);
    auto last_map_publish_time = std::chrono::steady_clock::now();
    unsigned long last_published_kf_id = 0;
    uint64_t last_registered_scan_seq = 0;
    CloudPtr accumulated_global_map{new PointCloudType()};

    while (rclcpp::ok() && !debug::flg_exit && !stop_viz_publish_thread_) {
        if (registered_scan_pub_ && registered_scan_pub_->get_subscription_count() > 0) {
            CloudPtr scan_copy = nullptr;
            SE3 scan_pose;
            uint64_t scan_seq = 0;
            {
                UL lock(viz_publish_mutex_);
                if (latest_registered_scan_seq_ > last_registered_scan_seq && latest_registered_scan_ != nullptr) {
                    scan_copy = latest_registered_scan_;
                    scan_pose = latest_registered_scan_pose_;
                    scan_seq = latest_registered_scan_seq_;
                }
            }

            if (scan_copy) {
                CloudPtr aligned(new PointCloudType);
                pcl::transformPointCloud(*scan_copy, *aligned, scan_pose.matrix());
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*aligned, msg);
                msg.header.stamp = node_->now();
                msg.header.frame_id = "map";
                registered_scan_pub_->publish(msg);
                last_registered_scan_seq = scan_seq;
            }
        }

        if (global_map_pub_ && has_kf_for_viz_.load() &&
            global_map_pub_->get_subscription_count() > 0) {

            const auto now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration<double>(now - last_map_publish_time).count();
            const unsigned long latest_kf_id = latest_kf_id_for_viz_.load();
            const bool force_publish = force_global_map_publish_.load();

            if (force_publish || (elapsed >= map_publish_interval_ && latest_kf_id != last_published_kf_id)) {
                CloudPtr batch = nullptr;
                bool full_replace = false;
                {
                    UL lock(viz_publish_mutex_);
                    if (global_map_cache_ && global_map_cache_->size() > 0) {
                        batch.swap(global_map_cache_);
                        global_map_cache_.reset(new PointCloudType());
                        full_replace = global_map_needs_full_replace_;
                        global_map_needs_full_replace_ = false;
                    }
                }

                if (batch && batch->size() > 0) {
                    if (full_replace) {
                        accumulated_global_map = batch;
                    } else {
                        *accumulated_global_map += *batch;
                    }

                    if (accumulated_global_map->size() > max_viz_points_) {
                        pcl::VoxelGrid<PointType> sor;
                        sor.setInputCloud(accumulated_global_map);
                        sor.setLeafSize(viz_voxel_size_, viz_voxel_size_, viz_voxel_size_);
                        CloudPtr filtered(new PointCloudType());
                        sor.filter(*filtered);
                        accumulated_global_map = filtered;
                    }

                    sensor_msgs::msg::PointCloud2 cloud_msg;
                    pcl::toROSMsg(*accumulated_global_map, cloud_msg);
                    cloud_msg.header.stamp = node_->now();
                    cloud_msg.header.frame_id = "map";
                    global_map_pub_->publish(cloud_msg);
                    last_published_kf_id = latest_kf_id;
                    last_map_publish_time = now;
                    force_global_map_publish_.store(false);
                }
            }
        }

        loop_rate.sleep();
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        if ((global_map_pub_ || registered_scan_pub_) && !viz_publish_thread_.joinable()) {
            stop_viz_publish_thread_ = false;
            viz_publish_thread_ = std::thread([this]() { PublishVisualizationLoop(); });
        }

        spin(node_);

        stop_viz_publish_thread_ = true;
        if (viz_publish_thread_.joinable()) {
            viz_publish_thread_.join();
        }
    }
}

}  // namespace lightning
