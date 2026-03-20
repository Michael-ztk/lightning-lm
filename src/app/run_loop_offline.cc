//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "ui/pangolin_window.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

#include <chrono>
#include <thread>

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);
    lightning::YAML_IO yaml(FLAGS_config);
    rosbag.SetImuInG(yaml.GetValue<bool>("common", "imu_in_g"));

    LaserMapping lio;
    if (!lio.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init lio";
        return -1;
    };

    auto ui = std::make_shared<ui::PangolinWindow>();
    ui->Init();
    lio.SetUI(ui);

    auto loop = std::make_shared<LoopClosing>();
    loop->Init(FLAGS_config);

    Keyframe::Ptr cur_kf = nullptr;

    rosbag
        .AddImuHandle("imu_raw",
                      [&lio](IMUPtr imu) {
                          lio.ProcessIMU(imu);
                          return true;
                      })
        .AddPointCloud2Handle("points_raw",
                              [&lio, &cur_kf, &loop](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  lio.ProcessPointCloud2(cloud);
                                  lio.Run();

                                  auto kf = lio.GetKeyframe();
                                  if (cur_kf != kf) {
                                      cur_kf = kf;
                                      loop->AddKF(kf);
                                  }

                                  return true;
                              })
        .Go();

    lio.SaveMap();
    Timer::PrintAll();

    while (!ui->ShouldQuit() && !debug::flg_exit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    LOG(INFO) << "done";

    return 0;
}
