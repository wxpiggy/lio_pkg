#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>
#include "common/point_types.h"

namespace wxpiggy {

/**
 * @brief ROS 发布器辅助类
 * 封装点云、位姿、路径等的发布功能
 */
class ROSPublisher {
public:
    /**
     * @brief 构造函数
     * @param nh ROS 节点句柄
     * @param cloud_topic 点云话题名称
     * @param odom_topic 里程计话题名称
     * @param path_topic 路径话题名称
     */
    ROSPublisher(ros::NodeHandle& nh, 
                 const std::string& cloud_topic = "/cloud_registered",
                 const std::string& odom_topic = "/odom",
                 const std::string& path_topic = "/odometry_path")
        : nh_(nh) {
        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic, 10);
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 100);
        pub_path_ = nh_.advertise<nav_msgs::Path>(path_topic, 5);
        
        path_.header.frame_id = "map";
    }

    /**
     * @brief 获取点云发布函数
     * @return 返回可用于 setFunc 的点云发布函数
     */
    std::function<bool(const std::string&, const FullCloudPtr&, double)> GetCloudPublishFunc() {
        return [this](const std::string& topic_name, const FullCloudPtr& cloud, double time) {
            return this->PublishCloud(cloud, time);
        };
    }
    

    std::function<bool(const std::string&, const CloudPtr&, double)> GetDownCloudPublishFunc(){
        return [this](const std::string& topic_name, const CloudPtr& cloud, double time) {
            return this->PublishCloud(cloud, time);
        };
    }

    /**
     * @brief 获取位姿发布函数
     * @return 返回可用于 setFunc 的位姿发布函数
     */
    std::function<bool(const std::string&, const SE3&, double)> GetPosePublishFunc() {
        return [this](const std::string& topic_name, const SE3& pose, double stamp) {
            return this->PublishPose(pose, stamp);
        };
    }

    /**
     * @brief 发布点云
     */
    bool PublishCloud(const FullCloudPtr& cloud, double time) {
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud, *cloud_msg);
        cloud_msg->header.stamp = ros::Time().fromSec(time);
        cloud_msg->header.frame_id = "map";
        pub_cloud_.publish(*cloud_msg);
        
        return true;
    }
    bool PublishCloud(const CloudPtr& cloud, double time) {
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud, *cloud_msg);
        cloud_msg->header.stamp = ros::Time().fromSec(time);
        cloud_msg->header.frame_id = "map";
        pub_cloud_.publish(*cloud_msg);
        
        return true;
    }
    /**
     * @brief 发布位姿（包括 TF、里程计、路径）
     */
    bool PublishPose(const SE3& pose, double stamp) {
        Eigen::Quaterniond q_current(pose.so3().matrix());
        Vec3d position = pose.translation();
        
        // 发布 TF 变换
        PublishTF(position, q_current, stamp);
        
        // 发布里程计
        PublishOdometry(position, q_current, stamp);
        
        // 发布路径
        PublishPath(position, q_current, stamp);
        
        return true;
    }

    /**
     * @brief 清空路径
     */
    void ClearPath() {
        path_.poses.clear();
    }

    /**
     * @brief 获取当前路径点数量
     */
    size_t GetPathSize() const {
        return path_.poses.size();
    }

private:
    /**
     * @brief 发布 TF 变换
     */
    void PublishTF(const Vec3d& position, const Eigen::Quaterniond& q, double stamp) {
        static tf::TransformBroadcaster br;
        
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
        tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
        transform.setRotation(tf_q);
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));
    }

    /**
     * @brief 发布里程计
     */
    void PublishOdometry(const Vec3d& position, const Eigen::Quaterniond& q, double stamp) {
        nav_msgs::Odometry odom;
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.header.stamp = ros::Time().fromSec(stamp);
        
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        
        odom.pose.pose.position.x = position.x();
        odom.pose.pose.position.y = position.y();
        odom.pose.pose.position.z = position.z();
        
        pub_odom_.publish(odom);
    }

    /**
     * @brief 发布路径
     */
    void PublishPath(const Vec3d& position, const Eigen::Quaterniond& q, double stamp) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = ros::Time().fromSec(stamp);
        
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        
        pose_stamped.pose.position.x = position.x();
        pose_stamped.pose.position.y = position.y();
        pose_stamped.pose.position.z = position.z();
        
        path_.header.stamp = ros::Time().fromSec(stamp);
        path_.poses.push_back(pose_stamped);
        pub_path_.publish(path_);
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_path_;
    nav_msgs::Path path_;
};

}  // namespace wxpiggy