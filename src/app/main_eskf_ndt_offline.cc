#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/console/print.h>

#include "slam/loosely_lio.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"
#include "ros_publisher.h"  // 新的头文件

DEFINE_string(bag_path, "/dataset/nclt/nclt9.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");
DEFINE_string(config, "/livox_ws/src/lio_pkg/config/velodyne_nclt.yaml", "path of config yaml");
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
    
    nh.param<std::string>("bag_path", bag_path, "/dataset/nclt/nclt9.bag");
    nh.param<std::string>("dataset_type", dataset_type, "NCLT");
    nh.param<std::string>("config", config_path, "$(find lio_pkg)/config/velodyne_nclt.yaml");

    // 创建 ROS 发布器（封装所有发布功能）
    wxpiggy::ROSPublisher ros_publisher(nh);

    // 初始化 RosbagIO
    wxpiggy::RosbagIO rosbag_io(fLS::FLAGS_bag_path, wxpiggy::Str2DatasetType(FLAGS_dataset_type));

    // 初始化 LooselyLIO
    wxpiggy::LooselyLIO::Options options;
    options.with_ui_ = FLAGS_display_map;
    wxpiggy::LooselyLIO lm(options);
    
    // 设置发布函数（简洁！）
    lm.setFunc(ros_publisher.GetCloudPublishFunc());
    lm.setFunc(ros_publisher.GetPosePublishFunc());
    lm.Init(FLAGS_config);

    LOG(INFO) << "LooselyLIO started, processing rosbag: " << FLAGS_bag_path;

    // 使用 RosbagIO 读取并处理数据
    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            wxpiggy::common::Timer::Evaluate([&]() { lm.PCLCallBack(cloud); }, "loosely lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            wxpiggy::common::Timer::Evaluate([&]() { lm.LivoxPCLCallBack(msg); }, "loosely lio");
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) {
            lm.IMUCallBack(imu);
            return true;
        })
        .Go();

    lm.Finish();
    wxpiggy::common::Timer::PrintAll();
    LOG(INFO) << "done. Total path points: " << ros_publisher.GetPathSize();

    return 0;
}