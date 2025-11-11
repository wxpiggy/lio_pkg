#pragma once
#include "common/nav_state.h"
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
// 从 GTSAM NavState 转换到自定义 NavState
namespace wxpiggy {
    inline wxpiggy::NavStated FromGTSAM(const gtsam::NavState& gtsam_state, double timestamp = 0) {
    wxpiggy::NavStated state;
    state.timestamp_ = timestamp;
    state.R_ = Sophus::SO3d(gtsam_state.pose().rotation().toQuaternion());
    state.p_ = gtsam_state.pose().translation();
    state.v_ = gtsam_state.velocity();
    // GTSAM NavState 不包含 bias，需要单独设置
    state.bg_ = Eigen::Vector3d::Zero();
    state.ba_ = Eigen::Vector3d::Zero();
    return state;
}

// 从 GTSAM Pose3 + Vector3 转换到自定义 NavState
inline wxpiggy::NavStated FromGTSAM(const gtsam::Pose3& pose, const gtsam::Vector3& velocity, 
                                   double timestamp = 0) {
    wxpiggy::NavStated state;
    state.timestamp_ = timestamp;
    state.R_ = Sophus::SO3d(pose.rotation().toQuaternion());
    state.p_ = pose.translation();
    state.v_ = velocity;
    state.bg_ = Eigen::Vector3d::Zero();
    state.ba_ = Eigen::Vector3d::Zero();
    return state;
}

// 从 GTSAM Pose3 + Vector3 + imuBias 转换到自定义 NavState
inline wxpiggy::NavStated FromGTSAM(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                                   const gtsam::imuBias::ConstantBias& bias, double timestamp = 0) {
    wxpiggy::NavStated state;
    state.timestamp_ = timestamp;
    state.R_ = Sophus::SO3d(pose.rotation().toQuaternion());
    state.p_ = pose.translation();
    state.v_ = velocity;
    state.bg_ = bias.gyroscope();
    state.ba_ = bias.accelerometer();
    return state;
}

// 从自定义 NavState 转换到 GTSAM NavState
inline gtsam::NavState ToGTSAMNavState(const wxpiggy::NavStated& state) {
    gtsam::Rot3 rot(state.R_.matrix());
    gtsam::Point3 trans(state.p_);
    gtsam::Pose3 pose(rot, trans);
    return gtsam::NavState(pose, state.v_);
}

// 从自定义 NavState 转换到 GTSAM Pose3
inline gtsam::Pose3 ToGTSAMPose(const wxpiggy::NavStated& state) {
    gtsam::Rot3 rot(state.R_.matrix());
    gtsam::Point3 trans(state.p_);
    return gtsam::Pose3(rot, trans);
}

// 从自定义 NavState 转换到 GTSAM imuBias
inline gtsam::imuBias::ConstantBias ToGTSAMBias(const wxpiggy::NavStated& state) {
    return gtsam::imuBias::ConstantBias(state.ba_, state.bg_);
}
}
