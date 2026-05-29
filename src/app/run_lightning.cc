#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/lightning_system.h"
#include "utils/timer.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);

    lightning::LightningSystem system;
    if (!system.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init LightningSystem";
        return -1;
    }

    system.Spin();

    lightning::Timer::PrintAll();
    rclcpp::shutdown();

    LOG(INFO) << "done";
    return 0;
}
