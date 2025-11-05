//
// Created by xiang on 2022/7/20.
//

#include "incremental_lo.h"

#include <pcl/common/transforms.h>

#include "core/registration/loam_icp.h"
#include "tools/math_utils.h"
#include "common/timer/timer.h"
#include "core/registration/ndt_inc.h"
#include "tools/config.h"

// #include "tools/pcl_map_viewer.h"
namespace wxpiggy {
void incrementalLO::Init(){
    auto config = Config::GetInstance().GetMappingConfig();
    registration_type_ = config.registration_type;
    if(registration_type_ == static_cast<int>(RegistrationBase::RegistraionType::LOAM)){
        LOG(INFO) << "using LOAM";
        // registration_ = std::make_unique<LoamICP>();
        // registration_->Init();
    }
    else if(registration_type_ == static_cast<int>(RegistrationBase::RegistraionType::NDT)){
        LOG(INFO) << "using Incremental NDT";
        registration_ = std::make_unique<IncNdt3d>();
        registration_->Init();
    }

}
void incrementalLO::AddCloud(CloudPtr scan, SE3& pose, bool use_guess) {
    
    if (first_frame_) {
        // 第一个帧，直接加入local map
        pose = SE3();
        last_kf_pose_ = pose;
        registration_->AddCloud({scan});
        first_frame_ = false;
        return;
    }
    SE3 guess;
    registration_->SetSource({scan});
    if (estimated_poses_.size() < 2) {
        registration_->Align(guess);
    } else {
        if (!use_guess) {
            // 从最近两个pose来推断
            SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
            SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
            guess = T1 * (T2.inverse() * T1);
        } else {
            guess = pose;
        }
        registration_->Align(guess);
    }
    pose = guess;
    estimated_poses_.emplace_back(pose);
    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, guess.matrix().cast<float>());
    if (IsKeyframe(pose)) {
        last_kf_pose_ = pose;
        cnt_frame_ = 0;
        // 放入ndt内部的local map
        // ndt_.AddCloud(scan_world);
        // icp_.AddCloud(scan_world);
        registration_->AddCloud({scan_world});
    }
    cnt_frame_++;
}

bool incrementalLO::IsKeyframe(const SE3& current_pose) {
    if (cnt_frame_ > 10) {
        return true;
    }

    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ || delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void incrementalLO::SaveMap(const std::string& map_path) {
    // if (viewer_) {
    //     viewer_->SaveMap(map_path);
    // }
}

}  // namespace wxpiggy