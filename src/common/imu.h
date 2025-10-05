//
// Created by xiang on 2021/7/19.
//

#pragma once

#include <memory>
#include "eigen_types.h"

namespace wxpiggy {

/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};
}  // namespace wxpiggy

using IMUPtr = std::shared_ptr<wxpiggy::IMU>;


