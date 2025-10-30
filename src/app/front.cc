#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <signal.h>
#include <atomic>

#include "frontend/frontend.h"

DEFINE_string(config_yaml, "/catkin_ws/src/lio_pkg/config/velodyne_nclt.yaml", "配置文件");

std::atomic<bool> shutdown_flag{false};

void SignalHandler(int signal) {
    LOG(INFO) << "Received signal: " << signal << ", shutting down Frontend...";
    shutdown_flag = true;
    ros::shutdown();
}

int main(int argc, char** argv) {
    // 初始化Google日志
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 初始化ROS
    ros::init(argc, argv, "frontend_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // 设置信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    LOG(INFO) << "==========================================";
    LOG(INFO) << "Starting Frontend Node";
    LOG(INFO) << "Config file: " << FLAGS_config_yaml;
    LOG(INFO) << "==========================================";

    try {
        // 创建前端实例
        auto frontend = std::make_shared<wxpiggy::Frontend>(FLAGS_config_yaml);

        // 初始化前端
        LOG(INFO) << "Initializing Frontend...";
        if (!frontend->Init(nh)) {
            LOG(ERROR) << "Failed to initialize Frontend";
            return -1;
        }
        LOG(INFO) << "Frontend initialized successfully";

        // 显示订阅的话题信息
        LOG(INFO) << "Subscribed topics:";
        LOG(INFO) << "  - PointCloud: /points_raw";
        LOG(INFO) << "  - IMU: /imu_raw"; 
        LOG(INFO) << "  - GNSS: /gps_rtk_fix";

        // 运行前端
        LOG(INFO) << "Starting Frontend processing...";
        frontend->Run();

        LOG(INFO) << "Frontend node shutdown complete";

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in Frontend node: " << e.what();
        return -1;
    } catch (...) {
        LOG(ERROR) << "Unknown exception in Frontend node";
        return -1;
    }

    return 0;
}