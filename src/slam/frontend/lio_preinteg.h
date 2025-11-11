#pragma once

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/Marginals.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include "common/nav_state.h"
#include "core/init/static_imu_init.h"
#include "preprocess/cloud_convert.h"
#include "preprocess/measure_sync.h"
#include "core/registration/ndt_inc.h"
#include "tools/math_utils.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace wxpiggy {

class LioPreinteg {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct Options {
        Options() {}
        bool verbose_ = true;

        double bias_gyro_var_ = 1e-4;
        double bias_acce_var_ = 1e-4;
        Mat3d bg_rw_info_ = Mat3d::Identity();
        Mat3d ba_rw_info_ = Mat3d::Identity();

        double ndt_pos_noise_ = 0.01;
        double ndt_ang_noise_ = 0.005;
        Mat6d lidar_pose_info_ = Mat6d::Identity();

        IncNdt3d::Options ndt_options_;
    };

    LioPreinteg();
    ~LioPreinteg() = default;

    bool Init();

    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);
    void IMUCallBack(IMUPtr msg_in);
    void Finish();

    using CloudPublishFunc = std::function<bool(const std::string&, const FullCloudPtr&, double)>;
    using CloudDownPublishFunc = std::function<bool(const std::string&, const CloudPtr&, double)>;
    using PosePublishFunc = std::function<bool(const std::string&, const SE3&, double)>;
    
    void setFunc(CloudPublishFunc func) { cloud_pub_func_ = func; }
    void setFunc(PosePublishFunc func) { pose_pub_func_ = func; }
    void setFunc(CloudDownPublishFunc func) { cloud_down_pub_func_ = func; }
    void setIMUfunc(PosePublishFunc func) { imu_pose_pub_func_ = func; }

   private:
    void ProcessMeasurements(const MeasureGroup& meas);
    void TryInitIMU();
    void Predict();
    void Undistort();
    void Align();
    void Optimize();
    void InitializeFixedLagSmoother();

    CloudPublishFunc cloud_pub_func_;
    PosePublishFunc pose_pub_func_, imu_pose_pub_func_;
    CloudDownPublishFunc cloud_down_pub_func_;
    std::string cloud_pub_topic_;   
    std::string pose_pub_topic_; 

    std::shared_ptr<MessageSync> sync_ = nullptr;
    StaticIMUInit imu_init_;

    wxpiggy::FullCloudPtr scan_undistort_{new FullPointCloudType()};
    CloudPtr current_scan_ = nullptr;

    NavStated current_nav_state_;
    NavStated last_nav_state_;
    
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegration_;
    IMUPtr last_imu_ = nullptr;

    std::shared_ptr<RegistrationBase> registration_;
    IncNdt3d ndt_;
    SE3 ndt_pose_;
    SE3 last_ndt_pose_;

    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;
    int frame_count_ = 0;
    
    MeasureGroup measures_;
    std::vector<NavStated> imu_states_;
    SE3 TIL_;
    Options options_;

    std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixed_lag_smoother_;
};

}  // namespace wxpiggy