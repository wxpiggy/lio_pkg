#include "gtsam_manager.h"
#include "navstate_utils.h"  // 包含你的转换函数

namespace wxpiggy {

GtsamManager::GtsamManager() {
    // 构造函数
}

bool GtsamManager::Initialize(const Options& options, const Vec3d& gravity, 
                            const NavStated& initial_state, const gtsam::imuBias::ConstantBias& initial_bias) {
    options_ = options;
    
    // 初始化固定滞后平滑器
    fixed_lag_smoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(options.lag_duration_);
    
    // 初始化IMU预积分参数
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(gravity.norm());
    p->n_gravity = gravity;
    p->integrationCovariance = 1e-8 * Eigen::Matrix3d::Identity();
    p->accelerometerCovariance = 0.01 * Eigen::Matrix3d::Identity();
    p->gyroscopeCovariance = 0.0001 * Eigen::Matrix3d::Identity();
    p->biasAccCovariance = options.bias_acce_var_ * options.bias_acce_var_ * Eigen::Matrix3d::Identity();
    p->biasOmegaCovariance = options.bias_gyro_var_ * options.bias_gyro_var_ * Eigen::Matrix3d::Identity();
    
    imu_preintegration_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, initial_bias);
    
    // 初始化激光雷达噪声模型
    double gp2 = options.ndt_pos_noise_ * options.ndt_pos_noise_;
    double ga2 = options.ndt_ang_noise_ * options.ndt_ang_noise_;
    Eigen::Matrix<double, 6, 1> lidar_sigmas;
    lidar_sigmas << 1.0/ga2, 1.0/ga2, 1.0/ga2, 1.0/gp2, 1.0/gp2, 1.0/gp2;
    lidar_pose_noise_ = gtsam::noiseModel::Gaussian::Information(lidar_sigmas.asDiagonal());
    
    // 初始化平滑器
    InitializeFixedLagSmoother(initial_state);
    
    frame_count_ = 1;
    return true;
}

void GtsamManager::InitializeFixedLagSmoother(const NavStated& initial_state) {
    using namespace gtsam;
    
    NonlinearFactorGraph new_factors;
    Values new_values;
    
    auto pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-2);
    auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-2);
    
    new_factors.addPrior(X(0), ToGTSAMPose(initial_state), pose_noise);
    new_factors.addPrior(V(0), initial_state.v_, velocity_noise);
    new_factors.addPrior(B(0), ToGTSAMBias(initial_state), bias_noise);
    
    new_values.insert(X(0), ToGTSAMPose(initial_state));
    new_values.insert(V(0), initial_state.v_);
    new_values.insert(B(0), ToGTSAMBias(initial_state));
    
    FixedLagSmoother::KeyTimestampMap new_timestamps;
    new_timestamps[X(0)] = initial_state.timestamp_;
    new_timestamps[V(0)] = initial_state.timestamp_;
    new_timestamps[B(0)] = initial_state.timestamp_;
    
    fixed_lag_smoother_->update(new_factors, new_values, new_timestamps);
}

void GtsamManager::IntegrateIMU(const IMU& imu, double dt) {
    if (imu_preintegration_) {
        imu_preintegration_->integrateMeasurement(imu.acce_, imu.gyro_, dt);
    }
}

NavStated GtsamManager::PredictState(const NavStated& last_state) const {
    if (!imu_preintegration_) {
        return last_state;
    }
    
    gtsam::NavState predicted_gtsam = imu_preintegration_->predict(
        ToGTSAMNavState(last_state), 
        ToGTSAMBias(last_state)
    );
    
    return FromGTSAM(
        predicted_gtsam.pose(), 
        predicted_gtsam.velocity(), 
        ToGTSAMBias(last_state),
        last_state.timestamp_ + imu_preintegration_->deltaTij()
    );
}

void GtsamManager::AddLidarMeasurement(const SE3& lidar_pose, double timestamp) {
    using namespace gtsam;
    
    int i = frame_count_ - 1;
    int j = frame_count_;
    
    NonlinearFactorGraph new_factors;
    Values new_values;

    // IMU因子
    // if (imu_preintegration_) {
        ImuFactor imu_factor(X(i), V(i), X(j), V(j), B(i), *imu_preintegration_);
        new_factors.add(imu_factor);

        // 偏置随机游走因子
        auto bias_noise_model = noiseModel::Diagonal::Sigmas(
            sqrt(imu_preintegration_->deltaTij()) *
            (Vector(6) << options_.bias_acce_var_, options_.bias_acce_var_, options_.bias_acce_var_,
             options_.bias_gyro_var_, options_.bias_gyro_var_, options_.bias_gyro_var_).finished()
        );

        new_factors.add(BetweenFactor<imuBias::ConstantBias>(
            B(i), B(j), imuBias::ConstantBias(), bias_noise_model));
    // }

    // 激光雷达先验因子
    Pose3 ndt_pose_gtsam(Rot3(lidar_pose.so3().matrix()), Point3(lidar_pose.translation()));
    new_factors.add(PriorFactor<Pose3>(X(j), ndt_pose_gtsam, lidar_pose_noise_));

    // 初始值
    if (imu_preintegration_) {
        gtsam::NavState predicted_gtsam = imu_preintegration_->predict(
            ToGTSAMNavState(GetLastState()), 
            ToGTSAMBias(GetLastState())
        );
        
        new_values.insert(X(j), predicted_gtsam.pose());
        new_values.insert(V(j), predicted_gtsam.velocity());
        new_values.insert(B(j), ToGTSAMBias(GetLastState()));
    }

    // 更新时间戳
    FixedLagSmoother::KeyTimestampMap new_timestamps;
    new_timestamps[X(j)] = timestamp;
    new_timestamps[V(j)] = timestamp;
    new_timestamps[B(j)] = timestamp;

    fixed_lag_smoother_->update(new_factors, new_values, new_timestamps);
    frame_count_++;
}



NavStated GtsamManager::GetCurrentState() const {
    if (!fixed_lag_smoother_) {
        return NavStated();
    }
    
    gtsam::Values result = fixed_lag_smoother_->calculateEstimate();
    int j = frame_count_ - 1;
    
    if (result.exists(X(j)) && result.exists(V(j)) && result.exists(B(j))) {
        return FromGTSAM(
            result.at<gtsam::Pose3>(X(j)), 
            result.at<gtsam::Vector3>(V(j)), 
            result.at<gtsam::imuBias::ConstantBias>(B(j)),
            0.0  // 时间戳需要外部设置
        );
    }
    
    return NavStated();
}

NavStated GtsamManager::GetLastState() const {
    if (!fixed_lag_smoother_) {
        return NavStated();
    }
    
    gtsam::Values result = fixed_lag_smoother_->calculateEstimate();
    int i = frame_count_ - 2;
    
    if (i >= 0 && result.exists(X(i)) && result.exists(V(i)) && result.exists(B(i))) {
        return FromGTSAM(
            result.at<gtsam::Pose3>(X(i)), 
            result.at<gtsam::Vector3>(V(i)), 
            result.at<gtsam::imuBias::ConstantBias>(B(i)),
            0.0  // 时间戳需要外部设置
        );
    }
    
    return NavStated();
}

gtsam::imuBias::ConstantBias GtsamManager::GetCurrentBias() const {
    if (!fixed_lag_smoother_) {
        return gtsam::imuBias::ConstantBias();
    }
    
    gtsam::Values result = fixed_lag_smoother_->calculateEstimate();
    int j = frame_count_ - 1;
    
    if (result.exists(B(j))) {
        return result.at<gtsam::imuBias::ConstantBias>(B(j));
    }
    
    return gtsam::imuBias::ConstantBias();
}

void GtsamManager::ResetIntegration() {
    if (imu_preintegration_) {
        auto optimized_bias = GetCurrentBias();
        imu_preintegration_->resetIntegrationAndSetBias(optimized_bias);
    }
}

}  // namespace wxpiggy