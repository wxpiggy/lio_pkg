//
// Created by xiang on 22-11-10.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "slam/lio_preinteg.h"
#include "common/io_utils.h"
// #include "common/sys_utils.h"
#include "common/timer/timer.h"
#include "ros_publisher.h"
DEFINE_string(bag_path, "/dataset/nclt/20120115.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "/livox_ws/src/lio_pkg/config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 屏蔽 PCL 警告
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    std::string bag_path, dataset_type, config_path;
    // 初始化 ROS 节点
    ros::init(argc, argv, "loosely_lio_offline");
    ros::NodeHandle nh;
    
    // nh.param<std::string>("bag_path", bag_path, "/dataset/nclt/nclt9.bag");
    // nh.param<std::string>("dataset_type", dataset_type, "NCLT");
    // nh.param<std::string>("config", config_path, "$(find lio_pkg)/config/velodyne_nclt.yaml");

    // 创建 ROS 发布器（封装所有发布功能）
    wxpiggy::ROSPublisher ros_publisher(nh);
    wxpiggy::RosbagIO rosbag_io(fLS::FLAGS_bag_path, wxpiggy::Str2DatasetType(FLAGS_dataset_type));

    wxpiggy::LioPreinteg lio;
    lio.setFunc(ros_publisher.GetCloudPublishFunc());
    lio.setFunc(ros_publisher.GetPosePublishFunc());
    lio.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            wxpiggy::common::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "Pre-Integration lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            wxpiggy::common::Timer::Evaluate([&]() { lio.LivoxPCLCallBack(msg); }, "Pre-Integration lio");
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) {
            lio.IMUCallBack(imu);
            return true;
        })
        .Go();

    lio.Finish();
    wxpiggy::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}