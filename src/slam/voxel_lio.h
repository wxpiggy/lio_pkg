#pragma once
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>

#include "core/eskf.hpp"
#include "core/static_imu_init.h"
#include "map/voxel_map.h"
#include "preprocess/measure_sync.h"

namespace wxpiggy {
namespace ui {
class PangolinWindow;
}

/**
 * 基于 Plane Voxel Map 的松耦合 LIO
 * 使用增量式平面体素地图进行配准
 * 相比 NDT，对平面特征更敏感，适合结构化环境
 */
class PlaneVoxelLIO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    struct Options {
        Options() {}
        bool save_motion_undistortion_pcd_ = false;  // 是否保存去畸变前后的点云
        bool with_ui_ = true;                        // 是否带着UI
        
        // Plane Voxel Map 参数
        double voxel_size_ = 1.0;              // 体素大小
        int max_points_per_voxel_ = 30;        // 每个体素最大点数
        double min_planarity_ = 0.01;           // 最小平面性阈值
        int min_points_for_plane_ = 5;         // 形成平面的最小点数
        size_t max_voxels_ = 500000;           // 最大体素数
        bool enable_plane_merge_ = false;       // 是否启用平面合并
        double merge_distance_th_ = 0.1;       // 合并距离阈值
        double merge_normal_th_ = 0.9;         // 合并法向量阈值
        
        // 下采样参数
        double leaf_size_ = 0.5;               // 体素下采样尺寸
    };

    PlaneVoxelLIO(Options options);
    ~PlaneVoxelLIO() = default;

    /// 从配置文件初始化
    bool Init(const std::string &config_yaml);

    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);

    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);

    /// 结束程序，退出UI
    void Finish();

    // 点云发布函数类型
    using CloudPublishFunc = std::function<bool(const std::string&, const FullCloudPtr&, double)>;
    // 位姿发布函数类型
    using PosePublishFunc = std::function<bool(const std::string&, const SE3&, double)>;
    
    // 设置发布函数
    void setFunc(CloudPublishFunc func) {
        cloud_pub_func_ = func;
    }
    
    void setFunc(PosePublishFunc func) {
        pose_pub_func_ = func;
    }

   private:
    CloudPublishFunc cloud_pub_func_;
    PosePublishFunc pose_pub_func_;

    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup &meas);

    /// 从YAML加载参数
    bool LoadFromYAML(const std::string &yaml);

    /// 尝试让IMU初始化
    void TryInitIMU();

    /// 利用IMU预测状态信息
    void Predict();

    /// 对measures_中的点云去畸变
    void Undistort();

    /// 执行一次配准和观测
    void Align();

   private:
    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;           // 消息同步器
    StaticIMUInit imu_init_;                                // IMU静止初始化
    std::shared_ptr<PlaneVoxelHashMap> plane_voxel_map_ = nullptr;  // 平面体素地图

    /// point clouds data
    FullCloudPtr scan_undistort_{new FullPointCloudType()};  // 去畸变后的点云
    SE3 pose_of_lo_;                                         // LO位姿

    Options options_;

    // flags
    bool imu_need_init_ = true;   // 是否需要估计IMU初始零偏
    bool flg_first_scan_ = true;  // 是否第一帧雷达
    int frame_num_ = 0;           // 帧数计数

    // EKF data
    MeasureGroup measures_;              // 同步之后的IMU和点云
    std::vector<NavStated> imu_states_;  // ESKF预测期间的状态
    ESKFD eskf_;                         // ESKF
    SE3 TIL_;                            // Lidar与IMU之间外参

    std::string cloud_pub_topic_;   
    std::string pose_pub_topic_; 
};

}  // namespace wxpiggy