#pragma once

#include <pcl/common/transforms.h>
#include <execution>
#include "core/init/static_imu_init.h"
#include "core/optimization/gtsam/gtsam_manager.h"
#include "core/registration/p2pl_icp.h"
#include "preprocess/measure_sync.h"
#include "tools/lidar_utils.h"
#include "tools/math_utils.h"
#include "tools/config.h"
#include "common/timer/timer.h"

namespace wxpiggy {

class LioPreinteg {
public:
    LioPreinteg();
    ~LioPreinteg() = default;

    bool Init();
    void ProcessMeasurements(const MeasureGroup &meas);
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void IMUCallBack(IMUPtr msg_in);
    void Finish();
        // 点云发布函数类型
    using CloudPublishFunc = std::function<bool(const std::string&, const FullCloudPtr&, double)>;
    // 位姿发布函数类型
    using PosePublishFunc = std::function<bool(const std::string&, const SE3&, double)>;
    using CloudDownPublishFunc = std::function<bool(const std::string&, const CloudPtr&, double)>;
    // 重载 setFunc
    void setFunc(CloudPublishFunc func) {
        cloud_pub_func_ = func;
    }
    
    void setFunc(PosePublishFunc func) {
        pose_pub_func_ = func;
    }
    void setFunc(CloudDownPublishFunc func){
        cloud_down_pub_func_ = func;
    }
private:
    void TryInitIMU();
    void Predict();
    void Undistort();
    void Align();
    void Optimize();
    void NormalizeVelocity();

private:
    std::unique_ptr<GtsamManager> gtsam_manager_;
    std::shared_ptr<StaticIMUInit> imu_init_;
    std::shared_ptr<MessageSync> sync_;
    std::shared_ptr<RegistrationBase> registration_;

    // 状态变量
    NavStated last_nav_state_;
    NavStated current_nav_state_;
    std::vector<NavStated> imu_states_;
    
    // 点云相关
    FullCloudPtr scan_undistort_;
    CloudPtr current_scan_;
    SE3 TIL_;  // 雷达到IMU的外参
    SE3 ndt_pose_;
    SE3 last_ndt_pose_;

    // 测量数据
    MeasureGroup measures_;
    IMUPtr last_imu_;

    // 标志位
    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;
    int frame_count_ = 0;

    // 配置选项
    struct {
        double bias_gyro_var_ = 0.001;
        double bias_acce_var_ = 0.01;
        double ndt_pos_noise_ = 0.1;
        double ndt_ang_noise_ = 0.05;
        bool verbose_ = false;
    } options_;

    // 发布函数
    std::function<void(const std::string&, FullCloudPtr, double)> cloud_pub_func_;
    std::function<void(const std::string&, CloudPtr, double)> cloud_down_pub_func_;
    std::function<void(const std::string&, const SE3&, double)> pose_pub_func_;
    std::string cloud_pub_topic_;
    std::string pose_pub_topic_;
};

}  // namespace wxpiggy