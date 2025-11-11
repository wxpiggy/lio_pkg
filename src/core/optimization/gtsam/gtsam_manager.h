#pragma once

#include "common/imu.h"
#include "common/nav_state.h"
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
// #include "common/types.h"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace wxpiggy {

class GtsamManager {
public:
    struct Options {
        double lag_duration_ = 1.0;
        double bias_gyro_var_ = 0.001;
        double bias_acce_var_ = 0.01;
        double ndt_pos_noise_ = 0.1;
        double ndt_ang_noise_ = 0.05;
    };

    GtsamManager();
    ~GtsamManager() = default;

    // 初始化
    bool Initialize(const Options& options, const Vec3d& gravity, 
                   const NavStated& initial_state, const gtsam::imuBias::ConstantBias& initial_bias);

    // IMU预积分
    void IntegrateIMU(const IMU& imu, double dt);
    NavStated PredictState(const NavStated& last_state) const;

    // 优化
    void AddLidarMeasurement(const SE3& lidar_pose, double timestamp);
    
    // 获取结果
    NavStated GetCurrentState() const;
    NavStated GetLastState() const;
    gtsam::imuBias::ConstantBias GetCurrentBias() const;

    // 重置
    void ResetIntegration();

private:
    void InitializeFixedLagSmoother(const NavStated& initial_state);

    Options options_;
    std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixed_lag_smoother_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegration_;
    
    int frame_count_ = 0;
    gtsam::noiseModel::Gaussian::shared_ptr lidar_pose_noise_;
};

}  // namespace wxpiggy