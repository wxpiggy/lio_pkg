#include "lio_preinteg.h"

namespace wxpiggy {

LioPreinteg::LioPreinteg() {
    // 初始化选项
    options_.bias_gyro_var_ = 0.001;
    options_.bias_acce_var_ = 0.01;
    options_.ndt_pos_noise_ = 0.1;
    options_.ndt_ang_noise_ = 0.05;
    options_.verbose_ = false;
}

bool LioPreinteg::Init() {
    imu_init_ = std::make_shared<StaticIMUInit>();
    imu_init_->Init();
    
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { 
        ProcessMeasurements(m); 
    });
    sync_->Init();
    
    registration_ = std::make_shared<IncIcp3d>();
    registration_->Init();
    
    cloud_pub_topic_ = "/cloud";
    pose_pub_topic_ = "/pose";
    
    auto mapping_config = Config::GetInstance().GetMappingConfig();
    Vec3d lidar_T_wrt_IMU = mapping_config.GetExtrinsicTranslation();
    Mat3d lidar_R_wrt_IMU = mapping_config.GetExtrinsicRotation();
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    
    return true;
}

void LioPreinteg::ProcessMeasurements(const MeasureGroup &meas) {
    LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
    measures_ = meas;

    if (imu_need_init_) {
        TryInitIMU();
        return;
    }

    Predict();
    Undistort();
    Align();
}

void LioPreinteg::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_->AddIMU(*imu);
    }

    if (!imu_init_->InitSuccess()) {
        return;
    }

    // 初始化GTSAM管理器
    GtsamManager::Options gtsam_options;
    gtsam_options.bias_gyro_var_ = options_.bias_gyro_var_;
    gtsam_options.bias_acce_var_ = options_.bias_acce_var_;
    gtsam_options.ndt_pos_noise_ = options_.ndt_pos_noise_;
    gtsam_options.ndt_ang_noise_ = options_.ndt_ang_noise_;
    
    gtsam_manager_ = std::make_unique<GtsamManager>();
    
    // 设置初始状态
    current_nav_state_.v_.setZero();
    current_nav_state_.bg_ = imu_init_->GetInitBg();
    current_nav_state_.ba_ = imu_init_->GetInitBa();
    current_nav_state_.timestamp_ = measures_.imu_.back()->timestamp_;
    
    // 初始化偏置
    gtsam::imuBias::ConstantBias prior_bias(current_nav_state_.ba_, current_nav_state_.bg_);
    
    bool init_success = gtsam_manager_->Initialize(
        gtsam_options, 
        imu_init_->GetGravity(), 
        current_nav_state_, 
        prior_bias
    );
    
    
    last_nav_state_ = current_nav_state_;
    last_imu_ = measures_.imu_.back();
    imu_need_init_ = false;
    
    LOG(INFO) << "IMU初始化成功";
}

void LioPreinteg::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(last_nav_state_);

    for (auto &imu : measures_.imu_) {
        if (last_imu_ != nullptr) {
            double dt = imu->timestamp_ - last_imu_->timestamp_;
            gtsam_manager_->IntegrateIMU(*imu, dt);
        }

        last_imu_ = imu;
        
        // 预测状态
        NavStated predicted_state = gtsam_manager_->PredictState(last_nav_state_);
        predicted_state.timestamp_ = imu->timestamp_;
        imu_states_.emplace_back(predicted_state);
    }
}

void LioPreinteg::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_state = imu_states_.back();
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), 
        [&](auto &pt) {
            SE3 Ti = T_end;
            NavStated match;

            math::PoseInterp<NavStated>(
                measures_.lidar_begin_time_ + pt.time * 1e-3, 
                imu_states_, 
                [](const NavStated &s) { return s.timestamp_; }, 
                [](const NavStated &s) { return s.GetSE3(); }, 
                Ti, match
            );

            Vec3d pi = ToVec3d(pt);
            Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

            pt.x = p_compensate(0);
            pt.y = p_compensate(1);
            pt.z = p_compensate(2);
        }
    );
    
    scan_undistort_ = cloud;
}

void LioPreinteg::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;

    current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);

    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);

    if (flg_first_scan_) {
        registration_->AddCloud({current_scan_filter});
        flg_first_scan_ = false;
        return;
    }

    LOG(INFO) << "=== frame " << frame_num_;
    registration_->SetSource({current_scan_filter});
    
    
    NavStated predicted_state = gtsam_manager_->PredictState(last_nav_state_);
    ndt_pose_ = predicted_state.GetSE3();
    
    registration_->Align(ndt_pose_);
    Optimize();

    SE3 current_pose = current_nav_state_.GetSE3();
    CloudPtr current_scan_world(new PointCloudType);
    pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());
    
    // 将点云添加到地图中
    registration_->AddCloud({current_scan_world});
    last_ndt_pose_ = current_pose;

    // 发布结果
    if (cloud_down_pub_func_) {
        cloud_down_pub_func_(cloud_pub_topic_, current_scan_world, measures_.lidar_end_time_);
    }
    if (pose_pub_func_) {
        pose_pub_func_(pose_pub_topic_, current_pose, measures_.lidar_end_time_);
    }
    
    frame_num_++;
}

void LioPreinteg::Optimize() {
    if (!gtsam_manager_) {
        return;
    }

    
    gtsam_manager_->AddLidarMeasurement(ndt_pose_, measures_.lidar_end_time_);
    
 
    
    // 获取优化后的状态
    current_nav_state_ = gtsam_manager_->GetCurrentState();
    current_nav_state_.timestamp_ = measures_.lidar_end_time_;
    
    last_nav_state_ = gtsam_manager_->GetLastState();
    last_nav_state_.timestamp_ = measures_.lidar_end_time_;
    
    // 重置预积分器
    gtsam_manager_->ResetIntegration();
    
    frame_count_++;

    if (options_.verbose_) {
        LOG(INFO) << "=== Frame " << frame_num_ << " ===";
        LOG(INFO) << "Position: " << current_nav_state_.p_.transpose();
        LOG(INFO) << "Velocity: " << current_nav_state_.v_.transpose();
        LOG(INFO) << "Bias - bg: " << current_nav_state_.bg_.transpose() 
                  << ", ba: " << current_nav_state_.ba_.transpose();
    }
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

void LioPreinteg::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (sync_) {
        sync_->ProcessCloud(msg);
    }
}

void LioPreinteg::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    if (sync_) {
        sync_->ProcessCloud(msg);
    }
}

void LioPreinteg::IMUCallBack(IMUPtr msg_in) {
    if (sync_) {
        sync_->ProcessIMU(msg_in);
    }
}

void LioPreinteg::Finish() {
    LOG(INFO) << "finish done";
}

}  // namespace wxpiggy