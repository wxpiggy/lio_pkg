//
// Created by xiang on 22-11-10.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "slam/lio_iekf.h"
#include "slam/voxel_lio.h"
#include "common/io_utils.h"
// #include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/dataset/nclt/20120115.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "/livox_ws/src/lio_pkg/config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    wxpiggy::RosbagIO rosbag_io(fLS::FLAGS_bag_path, wxpiggy::Str2DatasetType(FLAGS_dataset_type));
    // wxpiggy::LioIEKF lio;
    wxpiggy::VoxelLIO lio;
    lio.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            wxpiggy::common::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "IEKF lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            wxpiggy::common::Timer::Evaluate([&]() { lio.LivoxPCLCallBack(msg); }, "IEKF lio");
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
