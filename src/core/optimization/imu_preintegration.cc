#include "imu_preintegration.h"
#include <glog/logging.h>

namespace wxpiggy {

IMUPreintegration::IMUPreintegration(Options options) {
    bg_ = options.init_bg_;
    ba_ = options.init_ba_;
    const float ng2 = options.noise_gyro_ * options.noise_gyro_  ;
    const float na2 = options.noise_acce_ * options.noise_acce_;
    noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
}

void IMUPreintegration::Integrate(const IMU &imu, double dt) {
    if (!has_first_) {
        last_accel_ = imu.acce_;
        last_gyro_ = imu.gyro_;
        has_first_ = true;
        return;
    }

    // 去掉零偏的测量，使用平均值
    Vec3d gyr = (imu.gyro_ - bg_ + last_gyro_ - bg_) * 0.5;
    Vec3d acc = (imu.acce_ - ba_ + last_accel_ - ba_) * 0.5;

    // 先计算旋转增量
    Vec3d omega = gyr * dt;
    Mat3d rightJ = SO3::jr(omega);
    SO3 deltaR = SO3::exp(omega);

    // 运动方程雅可比矩阵系数 A,B
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();

    Mat3d acc_hat = SO3::hat(acc);
    double dt2 = dt * dt;

    // 设置A矩阵
    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
    A.block<3, 3>(6, 0) = -0.5 * dR_.matrix() * acc_hat * dt2;
    A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

    // 设置B矩阵
    B.block<3, 3>(0, 0) = rightJ * dt;
    B.block<3, 3>(3, 3) = dR_.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5 * dR_.matrix() * dt2;

    // 更新状态量 - 按正确顺序
    dp_ = dp_ + dv_ * dt + 0.5 * dR_.matrix() * acc * dt2;
    dv_ = dv_ + dR_ * acc * dt;
    dR_ = dR_ * deltaR;

    // 更新偏置雅可比
    dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;
    dV_dba_ = dV_dba_ - dR_.matrix() * dt;
    dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;
    dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5 * dR_.matrix() * dt2;
    dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5 * dR_.matrix() * dt2 * acc_hat * dR_dbg_;

    // 更新协方差
    cov_ = A * cov_ * A.transpose() + B * (noise_gyro_acce_ / dt) * B.transpose();
    
    // 添加积分噪声
    Mat3d i_cov = Mat3d::Identity() * 1e-8 * dt;
    cov_.block<3, 3>(6, 6) += i_cov;

    // 更新时间并保存当前数据
    dt_ += dt;
    last_accel_ = imu.acce_;
    last_gyro_ = imu.gyro_;
}

SO3 IMUPreintegration::GetDeltaRotation(const Vec3d &bg) { return dR_ * SO3::exp(dR_dbg_ * (bg - bg_)); }

Vec3d IMUPreintegration::GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba) {
    return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}

Vec3d IMUPreintegration::GetDeltaPosition(const Vec3d &bg, const Vec3d &ba) {
    return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}

NavStated IMUPreintegration::Predict(const wxpiggy::NavStated &start, const Vec3d &grav) const {
    SO3 Rj = start.R_ * dR_;
    Vec3d vj = start.R_ * dv_ + start.v_ + grav * dt_;
    Vec3d pj = start.R_ * dp_ + start.p_ + start.v_ * dt_ + 0.5f * grav * dt_ * dt_;

    auto state = NavStated(start.timestamp_ + dt_, Rj, pj, vj);
    state.bg_ = bg_;
    state.ba_ = ba_;
    return state;
}

}  // namespace sad