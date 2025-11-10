#include "lio_preinteg.h"

// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/core/robust_kernel.h>
// #include <g2o/core/sparse_block_matrix.h>
// #include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

#include <execution>
#include <fstream>
#include "core/optimization/gtsam/navstate_utils.h"
#include "core/registration/p2pl_icp.h"
// #include "core/optimization/g2o_types.h"
#include "tools/lidar_utils.h"
#include "tools/math_utils.h"
#include "tools/config.h"
#include "common/timer/timer.h"
#include <gtsam/inference/Symbol.h>
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
namespace wxpiggy {

LioPreinteg::LioPreinteg()  {
    
    gtsam::ISAM2Params isam_params;
isam_params.relinearizeThreshold = 0.01;
isam_params.relinearizeSkip = 1;
isam_params.findUnusedFactorSlots = true;
isam2_ = gtsam::ISAM2(isam_params);
    double bg_rw2 = 1.0 / options_.bias_gyro_var_ * options_.bias_gyro_var_;
    options_.bg_rw_info_.diagonal() << bg_rw2, bg_rw2, bg_rw2;
    double ba_rw2 = 1.0 / options_.bias_acce_var_ * options_.bias_acce_var_;
    options_.ba_rw_info_.diagonal() << ba_rw2, ba_rw2, ba_rw2;

    double gp2 = options_.ndt_pos_noise_ * options_.ndt_pos_noise_;
    double ga2 = options_.ndt_ang_noise_ * options_.ndt_ang_noise_;

    options_.lidar_pose_info_.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2, 1.0 / gp2, 1.0 / gp2;

    // options_.ndt_options_.nearby_type_ = IncNdt3d::NearbyType::CENTER;
    // ndt_ = IncNdt3d(options_.ndt_options_);
}



void LioPreinteg::ProcessMeasurements(const MeasureGroup &meas) {
    LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
    measures_ = meas;

    if (imu_need_init_) {
        // 初始化IMU系统
        TryInitIMU();
        return;
    }

    // 利用IMU数据进行状态预测
    Predict();

    // 对点云去畸变
    Undistort();

    // 配准
    Align();
}

bool LioPreinteg::Init() {
    // get params from yaml
  
    imu_init_ = StaticIMUInit();
    imu_init_.Init();
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });
    sync_->Init();
    registration_ = std::make_shared<IncIcp3d>();
    // registration_ = std::make_shared<IncNdt3d>();
    registration_->Init();
    cloud_pub_topic_ = "/cloud";
    pose_pub_topic_ = "/pose";
    /// 自身参数主要是雷达与IMU外参
    auto mapping_config = Config::GetInstance().GetMappingConfig();


    Vec3d lidar_T_wrt_IMU = mapping_config.GetExtrinsicTranslation();
    Mat3d lidar_R_wrt_IMU = mapping_config.GetExtrinsicRotation();
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    return true;
}

void LioPreinteg::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;

    current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);

    // voxel 之
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);

    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);

    /// the first scan
    if (flg_first_scan_) {
        registration_->AddCloud({current_scan_filter});
        // preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
        flg_first_scan_ = false;
        return;
    }

    // 后续的scan，使用NDT配合pose进行更新
    LOG(INFO) << "=== frame " << frame_num_;
    registration_->SetSource({current_scan_filter});
    auto state =  FromGTSAM(imu_preintegration_->predict(last_frame_, last_bias_));
    // current_nav_state_ = preinteg_->Predict(last_nav_state_, imu_init_.GetGravity());//重力一直是fixed？
    ndt_pose_ = current_nav_state_.GetSE3();

    registration_->Align(ndt_pose_);

    Optimize();

    // 若运动了一定范围，则把点云放入地图中
    SE3 current_pose = current_nav_state_.GetSE3();
    SE3 delta_pose = last_ndt_pose_.inverse() * current_pose;

    // if (delta_pose.translation().norm() > 0.5 || delta_pose.so3().log().norm() > math::deg2rad(10.0)) {
        // 将地图合入NDT中
        CloudPtr current_scan_world(new PointCloudType);
        pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());
        registration_->AddCloud({current_scan_world});
        last_ndt_pose_ = current_pose;

    // }


    FullCloudPtr scan_pub(new FullPointCloudType);        // 放入UI
    // pcl::transformPointCloud(*scan_undistort_,*scan_pub,current_pose.matrix());
    // cloud_pub_func_(cloud_pub_topic_,scan_pub,measures_.lidar_end_time_);
            // 放入UI
    cloud_down_pub_func_(cloud_pub_topic_,current_scan_world,measures_.lidar_end_time_);
    pose_pub_func_(pose_pub_topic_,current_pose,measures_.lidar_end_time_);
    frame_num_++;
}

void LioPreinteg::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {

        auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);

        // p->n_gravity = imu_init_.GetGravity();
        
        p->integrationCovariance = 1e-8 * Eigen::Matrix3d::Identity(); // integration uncertainty continuous
        // should be using 2nd order integration
        // PreintegratedRotation params:
         p->accelerometerCovariance = 0.01 * Eigen::Matrix3d::Identity();  // acc white noise in continuous
        p->gyroscopeCovariance = 0.0001 * Eigen::Matrix3d::Identity();// gyro white noise in continuous
        // p->accelerometerCovariance = imu_init_.GetCovAcce().asDiagonal();  // acc white noise in continuous
        // p->gyroscopeCovariance = imu_init_.GetCovGyro().asDiagonal();// gyro white noise in continuous
        p->biasAccCovariance = options_.bias_acce_var_ * options_.bias_acce_var_ * Eigen::Matrix3d::Identity();      // acc bias in continuous
        p->biasOmegaCovariance = options_.bias_gyro_var_ * options_.bias_gyro_var_ * Eigen::Matrix3d::Identity();  // gyro bias in continuous
        gtsam::imuBias::ConstantBias prior_imu_bias(imu_init_.GetInitBa(), imu_init_.GetInitBg()); 
        imu_preintegration_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p,prior_imu_bias);
        last_bias_ = prior_imu_bias;
        // preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
        imu_need_init_ = false;

        current_nav_state_.v_.setZero();
        current_nav_state_.bg_ = imu_init_.GetInitBg();
        current_nav_state_.ba_ = imu_init_.GetInitBa();
        current_nav_state_.timestamp_ = measures_.imu_.back()->timestamp_;

        last_nav_state_ = current_nav_state_;
        last_imu_ = measures_.imu_.back();
             gtsam::NonlinearFactorGraph init_factors;
    gtsam::Values init_values;
    
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e2);
    auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
    
    init_factors.addPrior(X(0), ToGTSAMPose(last_nav_state_), pose_noise);
    init_factors.addPrior(V(0), last_nav_state_.v_, velocity_noise);
    init_factors.addPrior(B(0), ToGTSAMBias(last_nav_state_), bias_noise);
    
    init_values.insert(X(0), ToGTSAMPose(last_nav_state_));
    init_values.insert(V(0), last_nav_state_.v_);
    init_values.insert(B(0), ToGTSAMBias(last_nav_state_));
    
    isam2_.update(init_factors, init_values);
    
    frame_count = 1;  // 重置帧计数
        LOG(INFO) << "IMU初始化成功";
    }
}

void LioPreinteg::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_state = imu_states_.back();  // 最后时刻的状态
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    /// 将所有点转到最后时刻状态上
    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto &pt) {
        SE3 Ti = T_end;
        NavStated match;

        // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
        math::PoseInterp<NavStated>(
            measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_, [](const NavStated &s) { return s.timestamp_; }, [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

        Vec3d pi = ToVec3d(pt);
        Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);
    });
    scan_undistort_ = cloud;
}

void LioPreinteg::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(last_nav_state_);

    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) {
        if (last_imu_ != nullptr) {
            imu_preintegration_->integrateMeasurement(imu->acce_, imu->gyro_, imu->timestamp_ - last_imu_->timestamp_);
            // preinteg_->Integrate(*imu, imu->timestamp_ - last_imu_->timestamp_);
        }

        last_imu_ = imu;
        // imu_preintegration_->predict(last_frame_, last_bias_);
        auto state = FromGTSAM(imu_preintegration_->predict(last_frame_, last_bias_));
        imu_states_.emplace_back(state);
        // imu_pose_pub_func_("/imu_pose",imu_states_.back().GetSE3(),imu_states_.back().timestamp_);
    }
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
    // if (ui_) {
    //     ui_->Quit();
    // }
    LOG(INFO) << "finish done";
}

void LioPreinteg::Optimize() {
    using namespace gtsam;
    
    static int frame_count = 0;
    frame_count++;
    
    int i = frame_count - 1;  // 上一帧
    int j = frame_count;      // 当前帧
    
    NonlinearFactorGraph new_factors;
    Values new_values;

    // 1. 添加 IMU 因子
    ImuFactor imu_factor(X(i), V(i), X(j), V(j), B(i), *imu_preintegration_);
    new_factors.add(imu_factor);

    // 2. 添加 bias 随机游走因子
    auto bias_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << options_.bias_acce_var_, options_.bias_acce_var_, options_.bias_acce_var_,
         options_.bias_gyro_var_, options_.bias_gyro_var_, options_.bias_gyro_var_).finished());

    new_factors.add(BetweenFactor<imuBias::ConstantBias>(
        B(i), B(j), imuBias::ConstantBias(), bias_noise_model));

    // 3. 添加 LiDAR 观测因子（NDT结果）
    Pose3 ndt_pose_gtsam(Rot3(ndt_pose_.so3().matrix()), Point3(ndt_pose_.translation()));
    new_factors.add(PriorFactor<Pose3>(
        X(j), ndt_pose_gtsam, noiseModel::Gaussian::Information(options_.lidar_pose_info_)));

    // 4. 添加新状态的初始值
    new_values.insert(X(j), ToGTSAMPose(current_nav_state_));
    new_values.insert(V(j), current_nav_state_.v_);
    new_values.insert(B(j), ToGTSAMBias(current_nav_state_));

    // 5. 更新 ISAM2
    isam2_.update(new_factors, new_values);
    
    // 6. 获取优化结果
    Values result = isam2_.calculateEstimate();

    // 7. 更新状态
    last_nav_state_ = FromGTSAM(
        result.at<Pose3>(X(i)), 
        result.at<Vector3>(V(i)), 
        result.at<imuBias::ConstantBias>(B(i)),
        last_nav_state_.timestamp_
    );

    current_nav_state_ = FromGTSAM(
        result.at<Pose3>(X(j)), 
        result.at<Vector3>(V(j)), 
        result.at<imuBias::ConstantBias>(B(j)),
        measures_.lidar_end_time_
    );

    // 8. 重置预积分器
    imuBias::ConstantBias current_bias = result.at<imuBias::ConstantBias>(B(j));
    imu_preintegration_->resetIntegrationAndSetBias(current_bias);

    // 9. 限制速度
    // NormalizeVelocity();

    // 10. 打印结果
    LOG(INFO) << "=== ISAM2 Frame " << frame_num_ << " ===";
    LOG(INFO) << "Position: " << current_nav_state_.p_.transpose();
    LOG(INFO) << "Velocity: " << current_nav_state_.v_.transpose();
    LOG(INFO) << "Bias - bg: " << current_nav_state_.bg_.transpose() 
              << ", ba: " << current_nav_state_.ba_.transpose();
}


void LioPreinteg::NormalizeVelocity() {
    /// 限制v的变化
    /// 一般是-y 方向速度
    Vec3d v_body = current_nav_state_.R_.inverse() * current_nav_state_.v_;
    if (v_body[1] > 0) {
        v_body[1] = 0;
    }
    v_body[2] = 0;

    if (v_body[1] < -2.0) {
        v_body[1] = -2.0;
    }

    if (v_body[0] > 0.1) {
        v_body[0] = 0.1;
    } else if (v_body[0] < -0.1) {
        v_body[0] = -0.1;
    }

    current_nav_state_.v_ = current_nav_state_.R_ * v_body;
}

}  // namespace wxpiggy