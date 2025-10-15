//
// Created by xiang on 2021/11/11.
//

#include "static_imu_init.h"
#include "common/math_utils.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace wxpiggy {

bool StaticIMUInit::AddIMU(const IMU& imu) {
    if (init_success_) {
        return true;
    }

    if (options_.use_speed_for_static_checking_ && !is_static_) {
        LOG(WARNING) << "等待车辆静止";
        init_imu_deque_.clear();
        return false;
    }

    if (init_imu_deque_.empty()) {
        // 记录初始静止时间
        init_start_time_ = imu.timestamp_;
    }

    // 记入初始化队列
    init_imu_deque_.push_back(imu);

    double init_time = imu.timestamp_ - init_start_time_;  // 初始化经过时间
    if (init_time > options_.init_time_seconds_) {
        // 尝试初始化逻辑
        TryInit();
    }

    // 维持初始化队列长度
    while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
        init_imu_deque_.pop_front();
    }

    current_time_ = imu.timestamp_;
    return false;
}

bool StaticIMUInit::AddOdom(const Odom& odom) {
    // 判断车辆是否静止
    if (init_success_) {
        return true;
    }

    if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
        is_static_ = true;
    } else {
        is_static_ = false;
    }

    current_time_ = odom.timestamp_;
    return true;
}

bool StaticIMUInit::TryInit() {
    if (init_imu_deque_.size() < 10) {
        return false;
    }

    // 计算均值和方差
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

    // 以acce均值为方向，取9.8长度为重力
    LOG(INFO) << "mean acce: " << mean_acce.transpose();
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

    // 重新计算加计的协方差
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });

    // 检查IMU噪声
    // if (cov_gyro_.norm() > options_.max_static_gyro_var) {
    //     LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
    //     return false;
    // }

    // if (cov_acce_.norm() > options_.max_static_acce_var) {
    //     LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var;
    //     return false;
    // }

    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() << ", gyro sq = " << cov_gyro_.transpose()
              << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
              << ", norm: " << gravity_.norm();
    LOG(INFO) << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose();
    init_success_ = true;
    return true;
}
void StaticIMUInit::LoadFromYaml(const std::string& config_file){
    auto yaml = YAML::LoadFile(config_file);
    double init_time_seconds_ = 2.0;                // 静止时间
    int init_imu_queue_max_size_ = 2000;            // 初始化IMU队列最大长度
    int static_odom_pulse_ = 5;                     // 静止时轮速计输出噪声
    double max_static_gyro_var = 0.5;               // 静态下陀螺测量方差
    double max_static_acce_var = 0.05;              // 静态下加计测量方差
    double gravity_norm_ = 9.81;                    // 重力大小
    bool use_speed_for_static_checking_ = true;     // 是否使用odom判断静止

    // 如果yaml中存在对应字段，就覆盖默认值
    if (yaml["init"]) {
        auto node = yaml["init"];
        if (node["init_time_seconds"]) init_time_seconds_ = node["init_time_seconds"].as<double>();
        if (node["init_imu_queue_max_size"]) init_imu_queue_max_size_ = node["init_imu_queue_max_size"].as<int>();
        if (node["static_odom_pulse"]) static_odom_pulse_ = node["static_odom_pulse"].as<int>();
        if (node["max_static_gyro_var"]) max_static_gyro_var = node["max_static_gyro_var"].as<double>();
        if (node["max_static_acce_var"]) max_static_acce_var = node["max_static_acce_var"].as<double>();
        if (node["gravity_norm"]) gravity_norm_ = node["gravity_norm"].as<double>();
        if (node["use_speed_for_static_checking"]) use_speed_for_static_checking_ = node["use_speed_for_static_checking"].as<bool>();
    }

    // 保存到成员变量
    options_.init_time_seconds_ = init_time_seconds_;
    options_.init_imu_queue_max_size_ = init_imu_queue_max_size_;
    options_.static_odom_pulse_ = static_odom_pulse_;
    options_.max_static_gyro_var = max_static_gyro_var;
    options_.max_static_acce_var = max_static_acce_var;
    options_.gravity_norm_ = gravity_norm_;
    options_.use_speed_for_static_checking_ = use_speed_for_static_checking_;
}
}  // namespace sad
