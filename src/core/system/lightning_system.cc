#include "core/system/lightning_system.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/localization/localization.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <opencv2/opencv.hpp>

namespace lightning {

LightningSystem::LightningSystem() { signal(SIGINT, lightning::debug::SigHandle); }

LightningSystem::~LightningSystem() {
    running_ = false;
    StopCurrentMode();
}

bool LightningSystem::Init(const std::string& yaml_path) {
    yaml_path_ = yaml_path;

    auto yaml = YAML::LoadFile(yaml_path);
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
    cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
    livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
    imu_in_g_ = yaml["common"]["imu_in_g"].as<bool>(false);

    rclcpp::QoS qos(10);

    // IMU 订阅
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            if (!running_) return;

            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity =
                Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            if (imu_in_g_) {
                imu->linear_acceleration *= 9.81;
            }

            ProcessIMU(imu);
        });

    // PointCloud2 订阅
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            if (!running_) return;
            Timer::Evaluate([&]() { ProcessLidarPC2(cloud); }, "Proc Lidar", true);
        });

    // Livox 订阅
    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
            if (!running_) return;
            Timer::Evaluate([&]() { ProcessLidarLivox(cloud); }, "Proc Lidar", false);
        });

    // /initialpose 订阅（定位模式用）
    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { OnInitialPose(msg); });

    // Publishers
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    if (yaml["system"]["publish_map_to_rviz"].as<bool>(false)) {
        map_publish_interval_ = yaml["system"]["map_publish_interval"].as<double>(1.0);
        global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", qos);
    }

    if (yaml["system"]["pub_registered_scan"].as<bool>(false)) {
        registered_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", qos);
    }

    rclcpp::QoS latching_qos(1);
    latching_qos.transient_local();
    static_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/static_map", latching_qos);

    // Service
    command_service_ = node_->create_service<srv::SystemCommand>(
        "lightning/command",
        [this](const srv::SystemCommand::Request::SharedPtr req, srv::SystemCommand::Response::SharedPtr res) {
            OnCommand(req, res);
        });

    LOG(INFO) << "LightningSystem initialized, mode: IDLE";
    return true;
}

// ======================== Service 回调 ========================

void LightningSystem::OnCommand(const srv::SystemCommand::Request::SharedPtr req,
                                 srv::SystemCommand::Response::SharedPtr res) {
    using Cmd = srv::SystemCommand::Request;
    using Res = srv::SystemCommand::Response;

    auto fill_status = [&]() {
        res->current_mode = static_cast<uint8_t>(current_mode_.load());
        switch (current_mode_.load()) {
            case Mode::IDLE:
                res->message += " [idle]";
                break;
            case Mode::MAPPING:
                res->message += " [mapping" + std::string(saving_map_ ? ", saving" : "") + "]";
                break;
            case Mode::LOCALIZING:
                res->message += std::string(" [localizing") + (loc_started_ ? "]" : ", waiting init pose]");
                break;
        }
    };

    switch (req->command) {
        case Cmd::CMD_GET_STATUS: {
            res->result = 0;
            res->message = current_map_name_;
            fill_status();
            return;
        }

        case Cmd::CMD_START_MAPPING: {
            if (current_mode_ == Mode::MAPPING) {
                res->result = 1;
                res->message = "already in mapping mode";
                fill_status();
                return;
            }

            running_ = false;
            if (save_thread_.joinable()) save_thread_.join();
            StopCurrentMode();

            if (StartMappingInternal(req->param)) {
                running_ = true;
                res->result = 0;
                res->message = "mapping started: " + req->param;
                LOG(INFO) << "mode switched to MAPPING: " << req->param;
            } else {
                res->result = 2;
                res->message = "failed to start mapping";
                LOG(ERROR) << res->message;
            }
            fill_status();
            return;
        }

        case Cmd::CMD_START_LOCALIZATION: {
            if (current_mode_ == Mode::LOCALIZING) {
                res->result = 1;
                res->message = "already in localization mode";
                fill_status();
                return;
            }

            std::string map_name = req->param.empty() ? current_map_name_ : req->param;
            if (map_name.empty()) {
                res->result = 2;
                res->message = "no map name specified";
                fill_status();
                return;
            }
            std::string map_path = "./data/" + map_name + "/";

            running_ = false;
            if (save_thread_.joinable()) save_thread_.join();
            StopCurrentMode();

            if (!std::filesystem::exists(map_path)) {
                res->result = 2;
                res->message = "map not found: " + map_path;
                LOG(ERROR) << res->message;
                fill_status();
                return;
            }

            if (StartLocalizationInternal(map_path)) {
                running_ = true;
                res->result = 0;
                res->message = "localization started, waiting for initial pose";
                LOG(INFO) << "mode switched to LOCALIZING: " << req->param;
            } else {
                res->result = 3;
                res->message = "failed to initialize localization";
                LOG(ERROR) << res->message;
            }
            fill_status();
            return;
        }

        case Cmd::CMD_STOP: {
            if (current_mode_ == Mode::IDLE) {
                res->result = 0;
                res->message = "already idle";
                fill_status();
                return;
            }

            running_ = false;
            if (save_thread_.joinable()) save_thread_.join();
            StopCurrentMode();

            res->result = 0;
            res->message = "stopped";
            LOG(INFO) << "mode switched to IDLE";
            fill_status();
            return;
        }

        case Cmd::CMD_SAVE_MAP: {
            if (current_mode_ != Mode::MAPPING) {
                res->result = 1;
                res->message = "not in mapping mode";
                fill_status();
                return;
            }

            if (saving_map_.load()) {
                res->result = 2;
                res->message = "save already in progress";
                fill_status();
                return;
            }

            std::string map_id = req->param.empty() ? current_map_name_ : req->param;
            std::string save_path = "./data/" + map_id + "/";

            if (save_thread_.joinable()) save_thread_.join();
            saving_map_ = true;
            save_thread_ = std::thread([this, save_path]() {
                SaveMapInternal(save_path);
                saving_map_ = false;
            });

            res->result = 0;
            res->message = "saving map to " + save_path;
            fill_status();
            return;
        }

        default: {
            res->result = 255;
            res->message = "unknown command: " + std::to_string(req->command);
            fill_status();
            return;
        }
    }
}

// ======================== 模式切换 ========================

bool LightningSystem::StartMappingInternal(const std::string& map_name) {
    auto yaml = YAML::LoadFile(yaml_path_);
    with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path_)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    if (with_loop_closing_) {
        LoopClosing::Options options;
        options.online_mode_ = true;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path_);
        lc_->SetLoopClosedCB([this]() {
            if (global_map_pub_) {
                auto global_map = lio_->GetGlobalMap(!with_loop_closing_, true, 0.3);
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

    if (with_visualization_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();
        lio_->SetUI(ui_);
    }

    if (with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = true;
        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path_);

        if (with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);
                if (step_on_kf_) {
                    cv::waitKey(0);
                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    // 如果定位模式下有创建 registered_scan_pub_，这里不需要额外处理
    // 定位模式使用 loc_ 的回调发布，建图模式用 viz 线程发布

    current_map_name_ = map_name;
    cur_kf_ = nullptr;

    // 重置可视化状态
    has_kf_for_viz_ = false;
    latest_kf_id_for_viz_ = 0;
    latest_registered_scan_ = nullptr;
    latest_registered_scan_seq_ = 0;
    global_map_cache_.reset(new PointCloudType());
    global_map_cache_seq_ = 0;
    global_map_needs_full_replace_ = false;
    force_global_map_publish_ = false;

    // 启动可视化线程
    if (global_map_pub_ || registered_scan_pub_) {
        stop_viz_publish_thread_ = false;
        viz_publish_thread_ = std::thread([this]() { PublishVisualizationLoop(); });
    }

    current_mode_ = Mode::MAPPING;
    return true;
}

bool LightningSystem::StartLocalizationInternal(const std::string& map_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    loc_->SetTFCallback(
        [this](const geometry_msgs::msg::TransformStamped& pose) { tf_broadcaster_->sendTransform(pose); });

    if (registered_scan_pub_) {
        loc_->SetRegisteredScanCallback([this](const sensor_msgs::msg::PointCloud2& registered_scan) {
            if (!registered_scan.data.empty()) {
                registered_scan_pub_->publish(registered_scan);
            }
        });
    }

    if (!loc_->Init(yaml_path_, map_path)) {
        LOG(ERROR) << "localization init failed";
        loc_.reset();
        return false;
    }

    std::string pcd_path = map_path + "/global.pcd";
    if (std::filesystem::exists(pcd_path) && static_map_pub_) {
        PublishStaticPCD(pcd_path);
    }

    loc_started_ = false;
    current_map_name_ = map_path;
    current_mode_ = Mode::LOCALIZING;
    return true;
}

void LightningSystem::StopCurrentMode() {
    if (current_mode_ == Mode::MAPPING) {
        stop_viz_publish_thread_ = true;
        if (viz_publish_thread_.joinable()) {
            viz_publish_thread_.join();
        }

        if (save_thread_.joinable()) {
            save_thread_.join();
        }

        lc_.reset();

        if (g2p5_) {
            g2p5_->Quit();
            g2p5_.reset();
        }

        if (ui_) {
            ui_->Quit();
            ui_.reset();
        }

        lio_.reset();
        cur_kf_ = nullptr;

        has_kf_for_viz_ = false;
        latest_kf_id_for_viz_ = 0;
        latest_registered_scan_ = nullptr;
        latest_registered_scan_seq_ = 0;
        global_map_cache_.reset(new PointCloudType());
        global_map_cache_seq_ = 0;
        global_map_needs_full_replace_ = false;
        force_global_map_publish_ = false;
        saving_map_ = false;

    } else if (current_mode_ == Mode::LOCALIZING) {
        if (loc_) {
            loc_->Finish();
            loc_.reset();
        }
        loc_started_ = false;
    }

    current_mode_ = Mode::IDLE;
}

// ======================== 传感器回调 ========================

void LightningSystem::ProcessIMU(const IMUPtr& imu) {
    Mode mode = current_mode_.load();
    if (mode == Mode::MAPPING) {
        lio_->ProcessIMU(imu);
    } else if (mode == Mode::LOCALIZING && loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LightningSystem::ProcessLidarPC2(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    Mode mode = current_mode_.load();
    if (mode == Mode::MAPPING) {
        lio_->ProcessPointCloud2(cloud);
        if (!lio_->Run()) return;
        PostLidarRunMapping();
    } else if (mode == Mode::LOCALIZING && loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LightningSystem::ProcessLidarLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    Mode mode = current_mode_.load();
    if (mode == Mode::MAPPING) {
        lio_->ProcessPointCloud2(cloud);
        if (!lio_->Run()) return;
        PostLidarRunMapping();
    } else if (mode == Mode::LOCALIZING && loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LightningSystem::PostLidarRunMapping() {
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

    if (cur_kf_ == nullptr) return;

    if (with_loop_closing_ && lc_) {
        lc_->AddKF(cur_kf_);
    }

    if (with_gridmap_ && g2p5_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    UpdateVisualizationCaches();
}

// ======================== 定位模式 ========================

void LightningSystem::OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg) {
    if (current_mode_ != Mode::LOCALIZING || !loc_) return;

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    LOG(INFO) << "received initial pose: pos=" << position.transpose()
              << ", quat=" << quaternion.coeffs().transpose();

    loc_->SetExternalPose(quaternion, position);
    loc_started_ = true;
}

void LightningSystem::PublishStaticPCD(const std::string& pcd_path) {
    try {
        CloudPtr global_map(new PointCloudType);
        if (pcl::io::loadPCDFile<PointType>(pcd_path, *global_map) == -1) {
            LOG(ERROR) << "failed to load PCD: " << pcd_path;
            return;
        }

        if (global_map->empty()) return;

        CloudPtr filtered_map = math::VoxelGrid(global_map, 0.2);
        if (filtered_map->empty()) return;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*filtered_map, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = node_->now();
        static_map_pub_->publish(msg);

        LOG(INFO) << "published static PCD: " << filtered_map->size() << "/" << global_map->size() << " points";
    } catch (const std::exception& e) {
        LOG(ERROR) << "exception publishing static PCD: " << e.what();
    }
}

// ======================== 建图可视化 ========================

void LightningSystem::PublishTF(const SE3& pose) {
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

void LightningSystem::UpdateVisualizationCaches() {
    latest_kf_id_for_viz_.store(cur_kf_->GetID());
    has_kf_for_viz_.store(true);

    const SE3 pose = with_loop_closing_ ? cur_kf_->GetOptBodyPose() : cur_kf_->GetLIOBodyPose();

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

void LightningSystem::PublishVisualizationLoop() {
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

        if (global_map_pub_ && has_kf_for_viz_.load() && global_map_pub_->get_subscription_count() > 0) {
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

// ======================== 保存地图 ========================

void LightningSystem::SaveMapInternal(const std::string& save_path) {
    LOG(INFO) << "saving map to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    auto global_map = lio_->GetGlobalMap(!with_loop_closing_);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptLidarPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);

    if (with_gridmap_ && g2p5_) {
        g2p5_->RedrawGlobalMap();
        while (!g2p5_->IsBusy()) {
            usleep(100000);
        }
        while (g2p5_->IsBusy()) {
            usleep(100000);
        }

        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {
                    nav_image.at<uchar>(height - 1 - y, x) = 255;
                } else if (data == 100) {
                    nav_image.at<uchar>(height - 1 - y, x) = 0;
                } else {
                    nav_image.at<uchar>(height - 1 - y, x) = 128;
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;
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

    LOG(INFO) << "map saved to " << save_path;
}

// ======================== Spin ========================

void LightningSystem::Spin() {
    if (node_) {
        spin(node_);

        running_ = false;
        StopCurrentMode();
    }
}

}  // namespace lightning
