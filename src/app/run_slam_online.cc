//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <vector>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char*> gflags_args;
    gflags_args.reserve(non_ros_args.size());
    for (auto& argument : non_ros_args) {
        gflags_args.emplace_back(argument.data());
    }
    int gflags_argc = static_cast<int>(gflags_args.size());
    char** gflags_argv = gflags_args.data();

    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv, true);

    using namespace lightning;

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);
    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");
    slam.Spin();

    Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}