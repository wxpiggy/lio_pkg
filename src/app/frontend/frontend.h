#pragma once

#include <map>
#include <memory>
#include <string>

#include "app/ros_publisher.h"
#include "common/gnss.h"
#include "common/nav_state.h"
#include "common/keyframe.h"
#include "ros/ros.h"

namespace wxpiggy {

class LioIEKF;
/**
 * 建图前端部分，将IMU和Lidar点云交给LIO处理，将RTK解析为rtk_pose
 */
class Frontend {
   public:
    struct Options {};

    // 从yaml文件中读取数据路径
    explicit Frontend(const std::string& config_yaml);

    // 初始化，创建LIO对象，检查数据存在性
    bool Init(ros::NodeHandle& nh);

    /// 运行前端
    void Run();
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void GnssCallback(const sensor_msgs::NavSatFixConstPtr& gnss_msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    // Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
    bool map_origin_set_ = false;
    std::mutex origin_mutex_;
    std::mutex gnss_mutex_;  // 保护gnss_数据的互斥锁
    
    // 在线原点设置方法
    void SetMapOriginOnline(GNSSPtr gnss);
    void SaveMapOriginToConfig();
    void ApplyOriginToHistoricalGNSS();
   private:
    /// 看某个state是否能提取到RTK pose
    void ExtractKeyFrame(const NavStated& state);

    /// 确定某个关键帧的GPS pose
    void FindGPSPose(std::shared_ptr<Keyframe> kf);

    /// 保存各个关键帧位姿，点云在生成时已经保存
    void SaveKeyframes();

    /// 提取RTK地图原点并移除此原点
    void RemoveMapOrigin();

    // 数据
    std::shared_ptr<Keyframe> last_kf_ = nullptr;            // 最近关键帧
    std::map<IdType, std::shared_ptr<Keyframe>> keyframes_;  // 抽取的关键帧
    std::shared_ptr<LioIEKF> lio_ = nullptr;                 // LIO
    std::string config_yaml_;                                // 配置文件路径
    std::shared_ptr<ROSPublisher> ros_publisher_;
    std::map<double, GNSSPtr> gnss_;  // GNSS 数据
    IdType kf_id_ = 0;                // 最新关键帧ID
    Vec3d map_origin_ = Vec3d::Zero();
    ros::Subscriber subPointCloud_;
    ros::Subscriber subImu_;
    ros::Subscriber subGnss_;
    // 参数和配置
    std::string bag_path_;         // 数据包路径
    std::string lio_yaml_;         // LIO 配置YAML
    double kf_dis_th_ = 1.0;       // 关键帧距离阈值
    double kf_ang_th_deg_ = 5.0;  // 关键帧角度阈值（度）
};

}  
