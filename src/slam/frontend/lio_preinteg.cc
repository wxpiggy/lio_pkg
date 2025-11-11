#include "lio_preinteg.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <pcl/common/transforms.h>

#include <execution>
#include "core/optimization/gtsam/navstate_utils.h"
#include "core/registration/p2pl_icp.h"
#include "tools/lidar_utils.h"
#include "tools/math_utils.h"
#include "tools/config.h"
#include "common/timer/timer.h"
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace wxpiggy {

LioPreinteg::LioPreinteg() {
    double lag = 10.0;
    fixed_lag_smoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(lag);
    
    double bg_rw2 = 1.0 / (options_.bias_gyro_var_ * options_.bias_gyro_var_);
    options_.bg_rw_info_.diagonal() << bg_rw2, bg_rw2, bg_rw2;
    double ba_rw2 = 1.0 / (options_.bias_acce_var_ * options_.bias_acce_var_);
    options_.ba_rw_info_.diagonal() << ba_rw2, ba_rw2, ba_rw2;

    double gp2 = options_.ndt_pos_noise_ * options_.ndt_pos_noise_;
    double ga2 = options_.ndt_ang_noise_ * options_.ndt_ang_noise_;
    options_.lidar_pose_info_.diagonal() << 1.0/ga2, 1.0/ga2, 1.0/ga2, 1.0/gp2, 1.0/gp2, 1.0/gp2;
}

bool LioPreinteg::Init() {
    imu_init_ = StaticIMUInit();
    imu_init_.Init();
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });
    sync_->Init();
    registration_ = std::make_shared<IncIcp3d>();
    registration_->Init();
    cloud_pub_topic_ = "/cloud";
    pose_pub_topic_ = "/pose";
    
    auto mapping_config = Config::GetInstance().GetMappingConfig();
    Vec3d lidar_T_wrt_IMU = mapping_config.GetExtrinsicTranslation();
    Mat3d lidar_R_wrt_IMU = mapping_config.GetExtrinsicRotation();
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    return true;
}

void LioPreinteg::ProcessMeasurements(const MeasureGroup &meas) {
    LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
    measures_ = meas;

    if (imu_need_init_) {
        TryInitIMU();
        return;
    }

    Predict();
    Undistort();
    Align();
}

void LioPreinteg::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (!imu_init_.InitSuccess()) {
        return;
    }

    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    p->n_gravity = imu_init_.GetGravity();
    p->integrationCovariance = 1e-8 * Eigen::Matrix3d::Identity();
    // p->accelerometerCovariance = imu_init_.GetCovAcce()[0] * Eigen::Matrix3d::Identity();
    // p->gyroscopeCovariance = imu_init_.GetCovGyro()[0] * Eigen::Matrix3d::Identity();
    p->accelerometerCovariance = 0.01 * Eigen::Matrix3d::Identity();
    p->gyroscopeCovariance = 0.0001 * Eigen::Matrix3d::Identity();
    p->biasAccCovariance = options_.bias_acce_var_ * options_.bias_acce_var_ * Eigen::Matrix3d::Identity();
    p->biasOmegaCovariance = options_.bias_gyro_var_ * options_.bias_gyro_var_ * Eigen::Matrix3d::Identity();
    gtsam::imuBias::ConstantBias prior_bias;
    // gtsam::imuBias::ConstantBias prior_bias(imu_init_.GetInitBa(), imu_init_.GetInitBg());
    imu_preintegration_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_bias);
    
    current_nav_state_.v_.setZero();
    current_nav_state_.bg_ = prior_bias.gyroscope();
    current_nav_state_.ba_ = prior_bias.accelerometer();
    current_nav_state_.timestamp_ = measures_.imu_.back()->timestamp_;
    
    last_nav_state_ = current_nav_state_;
    last_imu_ = measures_.imu_.back();
    
    imu_need_init_ = false;
    InitializeFixedLagSmoother();
    
    LOG(INFO) << "IMU初始化成功";
}

void LioPreinteg::InitializeFixedLagSmoother() {
    using namespace gtsam;
    
    NonlinearFactorGraph new_factors;
    Values new_values;
    
    auto pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-2);
    auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-2);
    
    new_factors.addPrior(X(0), ToGTSAMPose(last_nav_state_), pose_noise);
    new_factors.addPrior(V(0), last_nav_state_.v_, velocity_noise);
    new_factors.addPrior(B(0), ToGTSAMBias(last_nav_state_), bias_noise);
    
    new_values.insert(X(0), ToGTSAMPose(last_nav_state_));
    new_values.insert(V(0), last_nav_state_.v_);
    new_values.insert(B(0), ToGTSAMBias(last_nav_state_));
    
    gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps;
    new_timestamps[X(0)] = last_nav_state_.timestamp_;
    new_timestamps[V(0)] = last_nav_state_.timestamp_;
    new_timestamps[B(0)] = last_nav_state_.timestamp_;
    
    fixed_lag_smoother_->update(new_factors, new_values, new_timestamps);
    frame_count_ = 1;
}

void LioPreinteg::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(last_nav_state_);

    for (auto &imu : measures_.imu_) {
        if (last_imu_ != nullptr) {
            double dt = imu->timestamp_ - last_imu_->timestamp_;
            imu_preintegration_->integrateMeasurement(imu->acce_, imu->gyro_, dt);
        }

        last_imu_ = imu;
        
        gtsam::NavState predicted_gtsam = imu_preintegration_->predict(
            ToGTSAMNavState(last_nav_state_), 
            ToGTSAMBias(last_nav_state_)
        );
        
        NavStated predicted_state = FromGTSAM(
            predicted_gtsam.pose(), 
            predicted_gtsam.velocity(), 
            ToGTSAMBias(last_nav_state_),
            imu->timestamp_
        );
        
        imu_states_.emplace_back(predicted_state);
    }
}

void LioPreinteg::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_state = imu_states_.back();
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), 
        [&](auto &pt) {
            SE3 Ti = T_end;
            NavStated match;

            math::PoseInterp<NavStated>(
                measures_.lidar_begin_time_ + pt.time * 1e-3, 
                imu_states_, 
                [](const NavStated &s) { return s.timestamp_; }, 
                [](const NavStated &s) { return s.GetSE3(); }, 
                Ti, match
            );

            Vec3d pi = ToVec3d(pt);
            Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

            pt.x = p_compensate(0);
            pt.y = p_compensate(1);
            pt.z = p_compensate(2);
        }
    );
    
    scan_undistort_ = cloud;
}

void LioPreinteg::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;

    current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);

    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);

    if (flg_first_scan_) {
        registration_->AddCloud({current_scan_filter});
        flg_first_scan_ = false;
        return;
    }

    LOG(INFO) << "=== frame " << frame_num_;
    registration_->SetSource({current_scan_filter});
    
    gtsam::NavState predicted_gtsam = imu_preintegration_->predict(
        ToGTSAMNavState(last_nav_state_), 
        ToGTSAMBias(last_nav_state_)
    );
    ndt_pose_ = SE3(predicted_gtsam.pose().rotation().toQuaternion(), predicted_gtsam.pose().translation());
    
    registration_->Align(ndt_pose_);
    Optimize();

    SE3 current_pose = current_nav_state_.GetSE3();
    CloudPtr current_scan_world(new PointCloudType);
    pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());
    registration_->AddCloud({current_scan_world});
    last_ndt_pose_ = current_pose;

    cloud_down_pub_func_(cloud_pub_topic_, current_scan_world, measures_.lidar_end_time_);
    pose_pub_func_(pose_pub_topic_, current_pose, measures_.lidar_end_time_);
    frame_num_++;
}

void LioPreinteg::Optimize() {
    using namespace gtsam;
    
    int i = frame_count_ - 1;
    int j = frame_count_;
    
    NonlinearFactorGraph new_factors;
    Values new_values;

    ImuFactor imu_factor(X(i), V(i), X(j), V(j), B(i), *imu_preintegration_);
    new_factors.add(imu_factor);

    auto bias_noise_model = noiseModel::Diagonal::Sigmas(
        sqrt(imu_preintegration_->deltaTij()) *
        (Vector(6) << options_.bias_acce_var_, options_.bias_acce_var_, options_.bias_acce_var_,
         options_.bias_gyro_var_, options_.bias_gyro_var_, options_.bias_gyro_var_).finished()
    );

    new_factors.add(BetweenFactor<imuBias::ConstantBias>(
        B(i), B(j), imuBias::ConstantBias(), bias_noise_model));

    Pose3 ndt_pose_gtsam(Rot3(ndt_pose_.so3().matrix()), Point3(ndt_pose_.translation()));
    new_factors.add(PriorFactor<Pose3>(
        X(j), ndt_pose_gtsam, noiseModel::Gaussian::Information(options_.lidar_pose_info_)));

    gtsam::NavState predicted_gtsam = imu_preintegration_->predict(
        ToGTSAMNavState(last_nav_state_), 
        ToGTSAMBias(last_nav_state_)
    );
    
    new_values.insert(X(j), predicted_gtsam.pose());
    new_values.insert(V(j), predicted_gtsam.velocity());
    new_values.insert(B(j), ToGTSAMBias(last_nav_state_));

    gtsam::FixedLagSmoother::KeyTimestampMap lidar_pose_timestamps;
    lidar_pose_timestamps[X(j)] = measures_.lidar_end_time_;
    lidar_pose_timestamps[V(j)] = measures_.lidar_end_time_;
    lidar_pose_timestamps[B(j)] = measures_.lidar_end_time_;

    fixed_lag_smoother_->update(new_factors, new_values, lidar_pose_timestamps);
    
    Values result = fixed_lag_smoother_->calculateEstimate();

    if (result.exists(X(i)) && result.exists(V(i)) && result.exists(B(i))) {
        last_nav_state_ = FromGTSAM(
            result.at<Pose3>(X(i)), 
            result.at<Vector3>(V(i)), 
            result.at<imuBias::ConstantBias>(B(i)),
            last_nav_state_.timestamp_
        );
    }

    current_nav_state_ = FromGTSAM(
        result.at<Pose3>(X(j)), 
        result.at<Vector3>(V(j)), 
        result.at<imuBias::ConstantBias>(B(j)),
        measures_.lidar_end_time_
    );

    imuBias::ConstantBias optimized_bias = result.at<imuBias::ConstantBias>(B(j));
    imu_preintegration_->resetIntegrationAndSetBias(optimized_bias);
    
    frame_count_++;

    LOG(INFO) << "=== Frame " << frame_num_ << " ===";
    LOG(INFO) << "Position: " << current_nav_state_.p_.transpose();
    LOG(INFO) << "Velocity: " << current_nav_state_.v_.transpose();
    LOG(INFO) << "Bias - bg: " << current_nav_state_.bg_.transpose() 
              << ", ba: " << current_nav_state_.ba_.transpose();
}

void LioPreinteg::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

void LioPreinteg::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

void LioPreinteg::IMUCallBack(IMUPtr msg_in) {
    sync_->ProcessIMU(msg_in);
}

void LioPreinteg::Finish() {
    LOG(INFO) << "finish done";
}

}  // namespace wxpiggy