//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default_m20.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    // ROS launch appends --ros-args; remove those before gflags parses its flags.
    auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char*> pointers;
    for (auto& argument : arguments) pointers.push_back(argument.data());
    pointers.push_back(nullptr);
    int count = static_cast<int>(arguments.size());
    char** flags = pointers.data();
    google::ParseCommandLineFlags(&count, &flags, true);

    using namespace lightning;

    /// 需要rclcpp::init

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