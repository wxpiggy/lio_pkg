//
// Created by xiang on 22-8-15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "app/ros_publisher.h"
#include "slam/voxel_lio.h"
// #include "slam/loosely_lio.h"
#include "common/io_utils.h"
// #include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/dataset/nclt/nclt9.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "/livox_ws/src/lio_pkg/config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    wxpiggy::RosbagIO rosbag_io(fLS::FLAGS_bag_path, wxpiggy::Str2DatasetType(FLAGS_dataset_type));
        ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    wxpiggy::PlaneVoxelLIO::Options options;
    wxpiggy::ROSPublisher ros_publisher(nh);
    options.with_ui_ = FLAGS_display_map;
    wxpiggy::PlaneVoxelLIO lm(options);
    lm.setFunc(ros_publisher.GetCloudPublishFunc());
    lm.setFunc(ros_publisher.GetPosePublishFunc());
    lm.Init(FLAGS_config);
    
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
    LOG(INFO) << "done.";

    return 0;
}