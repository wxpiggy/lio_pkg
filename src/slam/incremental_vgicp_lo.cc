//
// Created by xiang on 2022/7/20.
//

#include "incremental_vgicp_lo.h"
#include "common/math_utils.h"
#include "common/timer/timer.h"

namespace wxpiggy {

void IncrementalVGICPLO::AddCloud(CloudPtr scan, SE3& pose, bool use_guess) {


            // return T_world_lidar;
    if (first_frame_) {
        // 第一个帧，直接加入local map
        pose = SE3();

        small_gicp::estimate_covariances_omp(scan,20,4);

        voxelmap->insert(scan);
        last_kf_pose_ = pose;
        first_frame_ = false;
        return;
    }

    // 此时local map位于NDT内部，直接配准即可
    SE3 guess;
    // ndt_.SetSource(scan);
    auto result = registration.align(voxelmap, scan, voxelmap);
    Eigen::Matrix3d R = result.T_target_source.rotation();    // 或 T_eigen.linear()
    Eigen::Vector3d t = result.T_target_source.translation();
    pose = Sophus::SE3d(R,t);
    voxelmap->insert(scan, result.T_target_source);
    // if (estimated_poses_.size() < 2) {
    //     ndt_.AlignNdt(guess);
    // } else {
    //     if (!use_guess) {
    //         // 从最近两个pose来推断
    //         SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
    //         SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
    //         guess = T1 * (T2.inverse() * T1);
    //     } else {
    //         guess = pose;
    //     }

    //     ndt_.AlignNdt(guess);
    // }

    // pose = guess;
    // estimated_poses_.emplace_back(pose);

    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());


    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
    cnt_frame_++;
}

bool IncrementalVGICPLO::IsKeyframe(const SE3& current_pose) {
    if (cnt_frame_ > 10) {
        return true;
    }

    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void IncrementalVGICPLO::SaveMap(const std::string& map_path) {
    if (viewer_) {
        viewer_->SaveMap(map_path);
    }
}

}  // namespace sad