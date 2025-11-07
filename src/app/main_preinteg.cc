#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/console/print.h>

#include "slam/frontend/lio_preinteg.h"
#include "common/timer/timer.h"
#include "ros_publisher.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>
#include "tools/config.h"


wxpiggy::LioPreinteg* lio = nullptr;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    wxpiggy::common::Timer::Evaluate([&]() { lio->PCLCallBack(cloud); }, "Pre-Integration LIO");
}

void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    wxpiggy::common::Timer::Evaluate([&]() { lio->LivoxPCLCallBack(msg); }, "Pre-Integration LIO");
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

    ros::init(argc, argv, "lio_preinteg_online");
    ros::NodeHandle nh;

    // 创建 ROS 发布器（封装所有发布功能）
    wxpiggy::ROSPublisher ros_publisher(nh);

    // 初始化 LioPreinteg
    std::string config_path;
    
    lio = new wxpiggy::LioPreinteg();
    
    nh.param<std::string>("config", config_path, "/project/src/lio_pkg/config/velodyne_nclt.yaml");
    Config::GetInstance().LoadConfig(config_path);
    // 设置发布函数
    lio->setFunc(ros_publisher.GetCloudPublishFunc());
    lio->setFunc(ros_publisher.GetPosePublishFunc());
    lio->setFunc(ros_publisher.GetDownCloudPublishFunc());
    lio->Init();

    // 根据数据集类型订阅点云
    ros::Subscriber subPointCloud;
    // if (FLAGS_dataset_type == "AVIA") {
    //     subPointCloud = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, livoxCallback);
    // } else {
    subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 100, pointCloudCallback);
    // }

    // IMU 订阅
    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("imu_raw", 500, imuCallback);

    LOG(INFO) << "LioPreinteg online node started, waiting for data...";

    ros::spin();

    lio->Finish();
    wxpiggy::common::Timer::PrintAll();
    LOG(INFO) << "done. Total path points: " << ros_publisher.GetPathSize();

    delete lio;
    return 0;
}