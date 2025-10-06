#include "voxel_lio.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include <execution>

#include "common/lidar_utils.h"
#include "common/point_cloud_utils.h"
#include "common/timer/timer.h"

namespace wxpiggy {

PlaneVoxelLIO::PlaneVoxelLIO(Options options) : options_(options) {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);
}

bool PlaneVoxelLIO::Init(const std::string &config_yaml) {
    /// 初始化自身的参数
    if (!LoadFromYAML(config_yaml)) {
        LOG(ERROR) << "Failed to load config from yaml";
        return false;
    }

    /// 初始化 Plane Voxel Map 的参数
    PlaneVoxelHashMap::Options plane_map_options;
    plane_map_options.voxel_size_ = options_.voxel_size_;
    plane_map_options.inv_voxel_size_ = 1.0 / options_.voxel_size_;
    plane_map_options.max_points_per_voxel_ = options_.max_points_per_voxel_;
    // plane_map_options.min_planarity_ = options_.min_planarity_;
    plane_map_options.min_points_for_plane_ = options_.min_points_for_plane_;
    plane_map_options.max_voxels_ = options_.max_voxels_;
    plane_map_options.enable_plane_merge_ = options_.enable_plane_merge_;
    plane_map_options.merge_distance_th_ = options_.merge_distance_th_;
    plane_map_options.merge_normal_th_ = options_.merge_normal_th_;
    plane_map_options.nearby_type_ = PlaneVoxelHashMap::NearbyType::NEARBY6;
    
    plane_voxel_map_ = std::make_shared<PlaneVoxelHashMap>(plane_map_options);

    LOG(INFO) << "PlaneVoxelLIO initialized successfully";
    LOG(INFO) << "  voxel_size: " << options_.voxel_size_;
    LOG(INFO) << "  max_points_per_voxel: " << options_.max_points_per_voxel_;
    LOG(INFO) << "  min_planarity: " << options_.min_planarity_;
    LOG(INFO) << "  enable_plane_merge: " << options_.enable_plane_merge_;

    return true;
}

bool PlaneVoxelLIO::LoadFromYAML(const std::string &yaml_file) {
    // 初始化消息同步器
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });
    sync_->Init(yaml_file);

    /// 读取外参
    auto yaml = YAML::LoadFile(yaml_file);
    std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
    
    cloud_pub_topic_ = "/cloud";
    pose_pub_topic_ = "/pose";
    
    Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
    Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    
    LOG(INFO) << "Loaded extrinsics T_IL: " << TIL_.translation().transpose();
    
    return true;
}

void PlaneVoxelLIO::ProcessMeasurements(const MeasureGroup &meas) {
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

void PlaneVoxelLIO::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(eskf_.GetNominalState());

    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) {
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
    }
}

void PlaneVoxelLIO::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        wxpiggy::ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        LOG(INFO) << "IMU初始化成功";
        LOG(INFO) << "  gyro_var: " << options.gyro_var_;
        LOG(INFO) << "  acce_var: " << options.acce_var_;
        LOG(INFO) << "  gravity: " << imu_init_.GetGravity().transpose();
    }
}

void PlaneVoxelLIO::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_state = eskf_.GetNominalState();  // 最后时刻的状态
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    if (options_.save_motion_undistortion_pcd_) {
        wxpiggy::SaveCloudToFile("./data/ch7/before_undist.pcd", *cloud);
    }

    /// 将所有点转到最后时刻状态上
    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto &pt) {
        SE3 Ti = T_end;
        NavStated match;

        // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
        math::PoseInterp<NavStated>(
            measures_.lidar_begin_time_ + pt.time * 1e-3,
            imu_states_,
            [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); },
            Ti,
            match);

        Vec3d pi = ToVec3d(pt);
        Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);
    });
    
    scan_undistort_ = cloud;

    if (options_.save_motion_undistortion_pcd_) {
        wxpiggy::SaveCloudToFile("./data/ch7/after_undist.pcd", *cloud);
    }
}

void PlaneVoxelLIO::Align() {
    common::Timer::Evaluate(
        [this]() {
            // 将点云转换到 Lidar 坐标系
            FullCloudPtr scan_undistort_trans(new FullPointCloudType);
            pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
            scan_undistort_ = scan_undistort_trans;

            // 转换为标准点云格式
            auto current_scan = ConvertToCloud<FullPointType>(scan_undistort_);

            // 体素下采样
            pcl::VoxelGrid<PointType> voxel;
            voxel.setLeafSize(options_.leaf_size_, options_.leaf_size_, options_.leaf_size_);
            voxel.setInputCloud(current_scan);

            CloudPtr current_scan_filter(new PointCloudType);
            voxel.filter(*current_scan_filter);

            LOG(INFO) << "Frame " << frame_num_ << ": original points: " << current_scan->size() 
                      << ", after voxel filter: " << current_scan_filter->size();

            /// 处理首帧雷达数据
            if (flg_first_scan_) {
                // 第一帧直接添加到地图，不进行配准
                plane_voxel_map_->AddCloud(current_scan);
                flg_first_scan_ = false;
                
                LOG(INFO) << "First scan added to map";
                LOG(INFO) << "  Total voxels: " << plane_voxel_map_->NumVoxels();
                LOG(INFO) << "  Valid planes: " << plane_voxel_map_->NumValidPlanes();
                return;
            }

            /// 从EKF中获取预测pose，放入Plane Voxel Map进行配准
            SE3 pose_predict = eskf_.GetNominalSE3();
            
            // 设置源点云
            plane_voxel_map_->SetSource(current_scan_filter);
            
            // 执行 ICP 配准
            bool align_success = plane_voxel_map_->AlignICP(pose_predict);
            
            if (!align_success) {
                LOG(WARNING) << "ICP alignment failed, using prediction";
            } else {
                LOG(INFO) << "ICP alignment success";
            }
            
            pose_of_lo_ = pose_predict;
            
            // 观测更新 EKF
            eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);
            SE3 pose_updated = eskf_.GetNominalSE3();
            
            // 将配准后的点云添加到地图
            CloudPtr current_scan_world(new PointCloudType);
            pcl::transformPointCloud(*current_scan_filter, *current_scan_world, pose_updated.matrix());
            plane_voxel_map_->AddCloud(current_scan_world);
            
            LOG(INFO) << "After adding scan:";
            LOG(INFO) << "  Total voxels: " << plane_voxel_map_->NumVoxels();
            LOG(INFO) << "  Valid planes: " << plane_voxel_map_->NumValidPlanes();

            // 发布点云和位姿
            if (cloud_pub_func_) {
                FullCloudPtr scan_pub(new FullPointCloudType);
                pcl::transformPointCloud(*scan_undistort_, *scan_pub, pose_updated.matrix());
                cloud_pub_func_(cloud_pub_topic_, scan_pub, measures_.lidar_end_time_);
            }
            
            if (pose_pub_func_) {
                pose_pub_func_(pose_pub_topic_, pose_updated, measures_.lidar_end_time_);
            }
            
            frame_num_++;
        },
        "Plane Voxel LIO Align");
}

void PlaneVoxelLIO::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

void PlaneVoxelLIO::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

void PlaneVoxelLIO::IMUCallBack(IMUPtr msg_in) {
    sync_->ProcessIMU(msg_in);
}

void PlaneVoxelLIO::Finish() {
    LOG(INFO) << "PlaneVoxelLIO finish";
    LOG(INFO) << "  Total frames processed: " << frame_num_;
    LOG(INFO) << "  Final map size: " << plane_voxel_map_->NumVoxels() << " voxels";
    LOG(INFO) << "  Valid planes: " << plane_voxel_map_->NumValidPlanes();
}

}  // namespace wxpiggy