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
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
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
    options_.save_lio_result_ = yaml["system"]["save_lio_result"].as<bool>(false);

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

        // IMU 订阅队列 depth 100 (=0.5s @200Hz)。原值 10 在 2x 回放/短暂停摆时丢采样，实测丢 ~1%
        // 即可让走廊段轨迹偏移 ~10cm 且逐轮不可复现；100 与 1000 的结果逐位一致，故取 100。
        rclcpp::QoS imu_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                if (imu_in_g_) {
                    imu->linear_acceleration *= 9.81;
                }

                // 使用 thread_local 避免多线程竞争（每个线程独立计数）
                thread_local double last_imu_header_time = -1.0;
                thread_local auto last_imu_callback_wall_time = std::chrono::steady_clock::time_point{};
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

        // 轮速里程计（可选话题）：底盘odom多为BestEffort发布（SensorDataQoS），
        // 订阅端也用best_effort避免QoS不匹配告警/收不到数据
        odom_topic_ = yaml["common"]["odom_topic"] ? yaml["common"]["odom_topic"].as<std::string>() : "";
        if (!odom_topic_.empty()) {
            rclcpp::QoS odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
            odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_, odom_qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                    // 轮速里程计：只用twist线速度（body系），pose漂移不可靠不使用
                    OdomPtr odom = std::make_shared<Odom>();
                    odom->timestamp_ = ToSec(msg->header.stamp);
                    odom->linear =
                        Vec3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
                    ProcessOdom(odom);
                });
            LOG(INFO) << "odom subscriber created, topic: " << odom_topic_;
        }

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
    // 路径安全校验：防止路径穿越攻击
    std::string sanitized_name = request->map_id;
    if (sanitized_name.find("..") != std::string::npos || sanitized_name.find('\0') != std::string::npos) {
        LOG(ERROR) << "invalid map name: contains unsafe characters";
        response->response = -1;
        return;
    }

    map_name_ = sanitized_name;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    // 空指针检查
    if (!lio_) {
        LOG(ERROR) << "cannot save map: LIO not initialized";
        return;
    }

    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    // 射线free-space清洗：位姿已最终确定（回环优化后），在导出前用全部关键帧的视线证据
    // 清除动态残影（行人等）与孤立噪点，直接改写关键帧点云，点云图与栅格图同时变干净
    lio_->RemoveDynamicByKeyframeRays();

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    // 全局点云地图保存分辨率(m)，低于此已小于雷达在 15~20 m 外的点间距，再细只是稀疏点不合并
    constexpr float kGlobalMapVoxel = 0.03f;
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_, true, kGlobalMapVoxel);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    auto kfs_all = lio_->GetAllKeyframes();
    if (kfs_all.empty()) {
        LOG(ERROR) << "cannot save map: no keyframes available";
        return;
    }
    // ConvertFromFullPCD() 内部断言地图非空：Debug 构建会直接中止进程，
    // Release 下断言被编译掉，会静默产出一张空的分块地图。这里提前拦截。
    if (!global_map || global_map->empty()) {
        LOG(ERROR) << "cannot save map: global map is empty";
        return;
    }

    // ===== 校验通过后才允许改动磁盘 =====
    // 下面会先清空目标目录，任何可能失败的校验都必须排在它前面，
    // 否则"保存失败"会连带把旧地图删掉（不可逆的数据丢失）。
    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = kfs_all.front()->GetOptLidarPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    // 前端位姿点云 global_lio.pcd，与后端位姿的 global.pcd 对比，用于区分前端漂移与后端优化不足
    if (options_.save_lio_result_ && options_.with_loop_closing_) {
        auto global_map_lio = lio_->GetGlobalMap(true, true, kGlobalMapVoxel);
        pcl::io::savePCDFileBinaryCompressed(save_path + "/global_lio.pcd", *global_map_lio);
    }

    // 保存被射线清洗过滤的动态点，用于误删检查
    lio_->SaveRayRemovedCloud(save_path);

    // 先用前端(LIO)位姿重建栅格图存为 map_lio.pgm，再走下方正常流程存 OptPose 的 map.pgm，
    // 两图对比定位重影来源：前端漂移(两图都重影) or 后端优化不足(仅 map.pgm 重影) or 后端引入(仅 map_lio 重影)
    if (options_.save_lio_result_ && options_.with_gridmap_ && options_.with_loop_closing_) {
        // 本工程（lightning-lm）的 G2P5 没有 SetKeyframes 接口，栅格重建直接读取其内部
        // 关键帧列表，因此只能沿用"临时把 OptPose 换成 LIO 位姿、重建后再还原"的方式。
        // 注意其固有风险：还原会覆盖期间回环线程写入的新优化结果；中途抛异常会跳过还原。
        // 这里仅补上等待超时，避免后端线程异常时无限阻塞保存流程。
        auto kfs_diag = lio_->GetAllKeyframes();
        std::vector<SE3> opt_backup;
        opt_backup.reserve(kfs_diag.size());
        for (const auto& kf : kfs_diag) {
            opt_backup.push_back(kf->GetOptLidarPose());
            kf->SetOptLidarPose(kf->GetLIOLidarPose());
        }

        g2p5_->RedrawGlobalMap();
        /// 后台线程最多 sleep(1) 才看到flag，给它最长10s开始；未开始就跳过，避免空等
        constexpr int kMaxWaitStartMs = 10000;  // 10秒等待启动
        constexpr int kMaxWaitBusyMs = 30000;   // 30秒等待完成
        int waited_ms = 0;

        for (int w = 0; w < 100 && !g2p5_->IsBusy(); ++w) {
            usleep(100000);
            waited_ms += 100;
        }

        if (!g2p5_->IsBusy()) {
            LOG(WARNING) << "g2p5 LIO redraw did not start within " << kMaxWaitStartMs
                        << "ms, skipping map_lio.pgm save";
        } else {
            waited_ms = 0;
            while (g2p5_->IsBusy() && waited_ms < kMaxWaitBusyMs) {
                usleep(100000);
                waited_ms += 100;
            }
            if (waited_ms >= kMaxWaitBusyMs) {
                LOG(ERROR) << "g2p5 LIO redraw timeout after " << kMaxWaitBusyMs << "ms";
            }
        }

        if (auto lio_grid = g2p5_->GetNewestMap()) {
            auto grid = lio_grid->ToROS();
            if (WriteGridPgm(grid, save_path + "/map_lio.pgm")) {
                LOG(INFO) << "front-end grid map saved: " << save_path << "/map_lio.pgm";
            }
        }

        for (size_t i = 0; i < kfs_diag.size(); ++i) {
            kfs_diag[i]->SetOptLidarPose(opt_backup[i]);
        }
    }

    // 导出TUM格式轨迹（LIO与回环优化后各一份），供evo评估对比
    if (options_.save_lio_result_) {
        auto kfs = lio_->GetAllKeyframes();
        std::ofstream f_lio(save_path + "/traj_lio.txt", std::ios::trunc);
        std::ofstream f_opt(save_path + "/traj_opt.txt", std::ios::trunc);
        f_lio << std::fixed << std::setprecision(6);
        f_opt << std::fixed << std::setprecision(6);
        for (const auto& kf : kfs) {
            double t = kf->GetState().timestamp_;
            auto write_one = [](std::ofstream& f, double ts, const SE3& pose) {
                const auto& q = pose.unit_quaternion();
                f << ts << " " << pose.translation().x() << " " << pose.translation().y() << " "
                  << pose.translation().z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            };
            write_one(f_lio, t, kf->GetLIOLidarPose());
            write_one(f_opt, t, kf->GetOptLidarPose());
        }
        LOG(INFO) << "trajectories saved: " << save_path << "/traj_lio.txt, traj_opt.txt (" << kfs.size() << " kfs)";
    }

    if (options_.with_gridmap_) {
        /// 使用优化后的位姿重建栅格地图。
        /// 等待方式用「完成计数」而非 is_busy_ 忙标志：is_busy_ 是瞬时脉冲，短数据集
        /// （如 825，96 个关键帧）渲染只需几十毫秒，100ms 轮询会整体错过 true->false
        /// 的窗口，被误判成"重绘未启动"而丢掉 map.pgm。
        const uint64_t render_before = g2p5_->GetRenderCount();
        g2p5_->RedrawGlobalMap();

        constexpr int kMaxWaitRenderMs = 60000;
        int waited_ms = 0;
        while (g2p5_->GetRenderCount() == render_before && waited_ms < kMaxWaitRenderMs) {
            usleep(10000);
            waited_ms += 10;
        }

        if (g2p5_->GetRenderCount() == render_before) {
            LOG(ERROR) << "g2p5 redraw did not complete within " << kMaxWaitRenderMs
                      << "ms, cannot save grid map";
            return;
        }

        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        WriteGridPgm(map, save_path + "/map.pgm");

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
        } catch (const YAML::Exception& e) {
            LOG(ERROR) << "YAML error while writing map.yaml: " << e.what();
            yamlFile.close();
            return;
        } catch (const std::exception& e) {
            LOG(ERROR) << "exception while writing map.yaml: " << e.what();
            yamlFile.close();
            return;
        } catch (...) {
            LOG(ERROR) << "unknown exception while writing map.yaml";
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

bool SlamSystem::WriteGridPgm(const nav_msgs::msg::OccupancyGrid& map, const std::string& pgm_path) {
    const int width = map.info.width;
    const int height = map.info.height;
    if (width <= 0 || height <= 0) {
        LOG(WARNING) << "empty grid map, skip " << pgm_path;
        return false;
    }

    cv::Mat nav_image(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        const int row_start_index = y * width;
        for (int x = 0; x < width; ++x) {
            int8_t data = map.data[row_start_index + x];
            if (data == 0) {                                   // Free
                nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
            } else if (data == 100) {                          // Occupied
                nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
            } else {                                           // Unknown
                nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
            }
        }
    }

    cv::imwrite(pgm_path, nav_image);
    return true;
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessOdom(const OdomPtr& odom) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessOdom(odom);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    // 对齐 rk100 的积压保护：缓冲过深说明处理落后于数据源，丢弃"新输入"以追上实时，
    // 但必须继续驱动 Run() 把已有缓冲排空——若在此直接 return，缓冲永远排不空，
    // 地图在某一时刻之后再也不更新（一次瞬时积压即触发，真机上同样会发生）。
    const bool too_deep = lio_->GetLidarBufferSize() > 2;
    if (too_deep) {
        static std::atomic<int> overflow_log_cnt{0};
        if (overflow_log_cnt.fetch_add(1) % 100 == 0) {
            LOG(WARNING) << "lidar buffer overflow, drop new scan (keep draining)";
        }
    } else {
        lio_->ProcessPointCloud2(cloud);
    }

    if (!lio_->Run()) {
        return;
    }
    auto state = lio_->GetState();
    if (state.pose_is_ok_) {
        PublishTF(state.GetPose());
    }

    Keyframe::Ptr kf;
    {
        std::unique_lock<std::mutex> lock(cur_kf_mutex_);
        auto new_kf = lio_->GetKeyframe();
        if (new_kf != cur_kf_) {
            cur_kf_ = new_kf;
            kf = cur_kf_;
        } else {
            return;
        }
    }

    if (kf == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(kf);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(kf);
    }

    if (ui_) {
        ui_->UpdateKF(kf);
    }

    UpdateVisualizationCaches();
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    // 同 PointCloud2 重载：积压时丢弃新输入但继续排空缓冲
    const bool too_deep = lio_->GetLidarBufferSize() > 2;
    if (too_deep) {
        static std::atomic<int> overflow_log_cnt2{0};
        if (overflow_log_cnt2.fetch_add(1) % 100 == 0) {
            LOG(WARNING) << "lidar buffer overflow (custom), drop new scan (keep draining)";
        }
    } else {
        lio_->ProcessPointCloud2(cloud);
    }

    if (!lio_->Run()) {
        return;
    }

    auto state = lio_->GetState();
    if (state.pose_is_ok_) {
        PublishTF(state.GetPose());
    }

    Keyframe::Ptr kf;
    {
        std::unique_lock<std::mutex> lock(cur_kf_mutex_);
        auto new_kf = lio_->GetKeyframe();
        if (new_kf != cur_kf_) {
            cur_kf_ = new_kf;
            kf = cur_kf_;
        } else {
            return;
        }
    }

    if (kf == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(kf);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(kf);
    }

    if (ui_) {
        ui_->UpdateKF(kf);
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
    Keyframe::Ptr kf;
    {
        std::unique_lock<std::mutex> lock(cur_kf_mutex_);
        kf = cur_kf_;
    }

    if (!kf) {
        return;
    }

    latest_kf_id_for_viz_.store(kf->GetID());
    has_kf_for_viz_.store(true);

    const SE3 pose = options_.with_loop_closing_ ? kf->GetOptBodyPose() : kf->GetLIOBodyPose();

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
        // 只入队，不做 PCL 运算：体素滤波+变换+累加在可视化线程里执行。
        // 原实现在此同步执行，阻塞点云回调并连带阻塞 IMU 回调（实测 wall_dt 达 130ms），
        // 造成 IMU 丢包与姿态精度下降。与 rk100 的后台实时地图线程方案对齐。
        UL lock(viz_publish_mutex_);
        pending_viz_kfs_.push_back(kf);
        while (pending_viz_kfs_.size() > 200) {
            pending_viz_kfs_.pop_front();  // 防止可视化落后时无限堆积
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
        // 先消化待处理关键帧：体素滤波 + 变换 + 累加到 global_map_cache_。
        // 这些 PCL 运算从点云回调移到这里，避免阻塞 IMU 回调。
        if (global_map_pub_) {
            std::deque<Keyframe::Ptr> batch_kfs;
            {
                UL lock(viz_publish_mutex_);
                batch_kfs.swap(pending_viz_kfs_);
            }

            for (auto& vkf : batch_kfs) {
                if (!vkf) {
                    continue;
                }
                CloudPtr cloud = vkf->GetCloud();
                if (!cloud || cloud->empty()) {
                    continue;
                }

                pcl::VoxelGrid<PointType> voxel;
                voxel.setLeafSize(1.0f, 1.0f, 1.0f);
                CloudPtr cloud_filter(new PointCloudType);
                voxel.setInputCloud(cloud);
                voxel.filter(*cloud_filter);

                const SE3 vpose = options_.with_loop_closing_ ? vkf->GetOptBodyPose() : vkf->GetLIOBodyPose();
                CloudPtr cloud_trans(new PointCloudType);
                pcl::transformPointCloud(*cloud_filter, *cloud_trans, vpose.matrix());
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
