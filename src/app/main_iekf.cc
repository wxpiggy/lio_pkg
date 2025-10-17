#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/console/print.h>

#include "slam/frontend/lio_iekf.h"
#include "common/timer/timer.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>
#include "ros_publisher.h"
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");
DEFINE_string(config, "/livox_ws/src/lio_pkg/config/velodyne_nclt.yaml", "path of config yaml");
DEFINE_bool(display_map, true, "display map?");

wxpiggy::LioIEKF* lio = nullptr;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*cloud_msg));
    wxpiggy::common::Timer::Evaluate([&]() { lio->PCLCallBack(cloud); }, "IEKF lio pcl");
}

void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    wxpiggy::common::Timer::Evaluate([&]() { lio->LivoxPCLCallBack(msg); }, "IEKF lio livox");
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    IMUPtr imu = std::make_shared<wxpiggy::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
    lio->IMUCallBack(imu);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 屏蔽 PCL 警告
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    ros::init(argc, argv, "lio_iekf_online");
    ros::NodeHandle nh;
    wxpiggy::ROSPublisher ros_publisher(nh);
    // 配置参数
    std::string config_path;
    nh.param<std::string>("config", config_path, "111");
    std::cout << config_path << "=======================";
    // 初始化 LioIEKF

    lio = new  wxpiggy::LioIEKF();
    lio ->setFunc(ros_publisher.GetCloudPublishFunc());
    lio ->setFunc(ros_publisher.GetPosePublishFunc());
    lio->Init(config_path);

    // std::string dataset_type;
    // nh.param<std::string>("dataset_type", dataset_type, FLAGS_dataset_type);

    ros::Subscriber subPointCloud;

    subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 100, pointCloudCallback);


    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("imu_raw", 1000, imuCallback);

    LOG(INFO) << "LioIEKF online node started, waiting for data...";

    ros::spin();

    lio->Finish();
    wxpiggy::common::Timer::PrintAll();
    LOG(INFO) << "done.";
    delete lio;
    return 0;
}