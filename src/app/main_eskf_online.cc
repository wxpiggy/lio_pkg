#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/console/print.h>

#include "slam/frontend/loosely_eskf/loosely_lio.h"
// #include "common/timer/timer.h"
#include "ros_publisher.h"  // 新的头文件
#include "tools/config.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>

wxpiggy::LooselyLIO* lm = nullptr;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    // wxpiggy::common::Timer::Evaluate([&]() { 
        lm->PCLCallBack(cloud); 
    // }, "loosely lio");
}

void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    // wxpiggy::common::Timer::Evaluate([&]() { 
        lm->LivoxPCLCallBack(msg); 
    // }, "loosely lio");
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    IMUPtr imu = std::make_shared<wxpiggy::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
    lm->IMUCallBack(imu);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    // 屏蔽 PCL 警告
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    std::string bag_path, config_path, lidar_topic, imu_topic;
    ros::init(argc, argv, "loosely_lio");
    ros::NodeHandle nh;
    nh.param<std::string>("config", config_path, "/project/src/lio_pkg/config/velodyne_nclt.yaml");
    Config::GetInstance().LoadConfig(config_path);
    Config::SystemConfig system_config;
    system_config = Config::GetInstance().getSystemConfig();
    wxpiggy::ROSPublisher ros_publisher(nh);
    lm = new wxpiggy::LooselyLIO();
    lm->setFunc(ros_publisher.GetCloudPublishFunc());
    lm->setFunc(ros_publisher.GetPosePublishFunc());
    lm->setFunc(ros_publisher.GetDownCloudPublishFunc());
    lm->Init();
    ros::Subscriber subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(system_config.lidar_topic, 1000, pointCloudCallback);
    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(system_config.imu_topic, 5000, imuCallback);
    LOG(INFO) << "LooselyLIO node started, waiting for data...";
    ros::spin();
    lm->Finish();
    // wxpiggy::common::Timer::PrintAll();
    LOG(INFO) << "done. Total path points: " << ros_publisher.GetPathSize();

    delete lm;

    return 0;
}