//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default_m20.yaml", "配置文件");

/// 运行定位的测试
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


    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
        return -1;
    }

    /// 默认起点开始定位
    loc.SetInitPose(SE3());
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}