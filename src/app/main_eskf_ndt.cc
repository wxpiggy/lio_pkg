#include <gflags/gflags.h>
#include <glog/logging.h>

#include "slam/loosely_lio.h"
#include "common/timer/timer.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");
DEFINE_string(config, "/livox_ws/src/lio_pkg/config/velodyne_nclt.yaml", "path of config yaml");
DEFINE_bool(display_map, true, "display map?");

wxpiggy::LooselyLIO* lm = nullptr;
nav_msgs::Path odomPath;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    wxpiggy::common::Timer::Evaluate([&]() { 
        lm->PCLCallBack(cloud); 
    }, "loosely lio");
}

void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    wxpiggy::common::Timer::Evaluate([&]() { 
        lm->LivoxPCLCallBack(msg); 
    }, "loosely lio");
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
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

    ros::init(argc, argv, "loosely_lio");
    ros::NodeHandle nh;

    // 创建发布器
    ros::Publisher pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);
    ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);

    // 点云发布函数
    auto cloud_pub_func = std::function<bool(const std::string& topic_name, const wxpiggy::FullCloudPtr& cloud, double time)>(
        [&](const std::string& topic_name,const wxpiggy::FullCloudPtr& cloud, double time) {
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);
            cloud_ptr_output->header.stamp = ros::Time().fromSec(time);
            cloud_ptr_output->header.frame_id = "map";
            pubPointCloud.publish(*cloud_ptr_output);
            return true;
        }
    );

    // 位姿发布函数
    auto pose_pub_func = std::function<bool(const std::string& topic_name, const SE3& pose, double stamp)>(
        [&](const std::string& topic_name, const SE3& pose, double stamp) {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

            // 发布里程计
            nav_msgs::Odometry odom;
            odom.header.frame_id = "map";
            odom.child_frame_id = "base_link";
            odom.header.stamp = ros::Time().fromSec(stamp);
            odom.pose.pose.orientation.x = q_current.x();
            odom.pose.pose.orientation.y = q_current.y();
            odom.pose.pose.orientation.z = q_current.z();
            odom.pose.pose.orientation.w = q_current.w();
            odom.pose.pose.position.x = pose.translation().x();
            odom.pose.pose.position.y = pose.translation().y();
            odom.pose.pose.position.z = pose.translation().z();
            pubOdometry.publish(odom);

            // 发布路径
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header = odom.header;
            poseStamped.pose = odom.pose.pose;
            odomPath.header.stamp = odom.header.stamp;
            odomPath.header.frame_id = "map";
            odomPath.poses.push_back(poseStamped);
            pubPath.publish(odomPath);

            return true;
        }
    );

    wxpiggy::LooselyLIO::Options options;
    options.with_ui_ = FLAGS_display_map;
    lm = new wxpiggy::LooselyLIO(options);
        // 设置发布函数
    lm->setFunc(cloud_pub_func);
    lm->setFunc(pose_pub_func);
    lm->Init(FLAGS_config);


    // 根据数据集类型订阅不同的点云话题
    ros::Subscriber subPointCloud;
    std::string dataset_type = FLAGS_dataset_type;
    
    if (dataset_type == "AVIA") {
        subPointCloud = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, livoxCallback);
    } else {
        subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 100, pointCloudCallback);
    }

    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("imu_raw", 500, imuCallback);

    std::cout << "LooselyLIO node started, waiting for data..." << std::endl;

    ros::spin();

    lm->Finish();
    wxpiggy::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    delete lm;

    return 0;
}