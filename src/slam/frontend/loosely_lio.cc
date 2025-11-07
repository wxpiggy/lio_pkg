#include "loosely_lio.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include <execution>

#include "tools/lidar_utils.h"
#include "tools/point_cloud_utils.h"
#include "common/timer/timer.h"
#include "tools/config.h"
// #include "tools/ui/pangolin_window.h"
namespace wxpiggy {



bool LooselyLIO::Init() {
    
    eskf_ = std::make_shared<ESKFD>();
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit();
    imu_init_.Init();
    /// 初始化NDT LO的参数
    wxpiggy::incrementalLO::Options incLO_options;
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });
    sync_->Init();
    auto mapping_config = Config::GetInstance().GetMappingConfig();
    cloud_pub_topic_ = "/cloud";
    pose_pub_topic_ = "/pose";
    Vec3d lidar_T_wrt_IMU = mapping_config.GetExtrinsicTranslation();
    Mat3d lidar_R_wrt_IMU = mapping_config.GetExtrinsicRotation();
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    inc_lo_ = std::make_shared<wxpiggy::incrementalLO>();
    inc_lo_->Init();
    return true;
}

// bool LooselyLIO::LoadFromYAML(const std::string &yaml_file) {
//     // get params from yaml
//     sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });
//     sync_->Init(yaml_file);

//     /// 自身参数主要是雷达与IMU外参
//     auto yaml = YAML::LoadFile(yaml_file);
//     std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
//     std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
//     cloud_pub_topic_ = "/cloud";
//     pose_pub_topic_ = "/pose";
//     Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
//     Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
//     TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
//     return true;
// }

void LooselyLIO::ProcessMeasurements(const MeasureGroup &meas) {
    // LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
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

void LooselyLIO::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(eskf_->GetNominalState());

    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) {
        eskf_->Predict(*imu);
        imu_states_.emplace_back(eskf_->GetNominalState());
    }
}

void LooselyLIO::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        wxpiggy::ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        eskf_->SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        LOG(INFO) << "IMU初始化成功";
    }
}

void LooselyLIO::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_state = eskf_->GetNominalState();  // 最后时刻的状态
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);
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

    // if (options_.save_motion_undistortion_pcd_) {
    //     wxpiggy::SaveCloudToFile("./data/ch7/after_undist.pcd", *cloud);
    // }
}

void LooselyLIO::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
    scan_undistort_ = scan_undistort_trans;
    auto current_scan = ConvertToCloud<FullPointType>(scan_undistort_);
    /// 处理首帧雷达数据
    if (flg_first_scan_) {
        SE3 pose;
        inc_lo_->AddCloud(current_scan, pose);
        flg_first_scan_ = false;
        return;
    }
    auto current_scan_filter = VoxelCloud(current_scan,0.5);
    SE3 pose_predict = eskf_->GetNominalSE3();
    inc_lo_->AddCloud(current_scan_filter, pose_predict, true);
    pose_of_lo_ = pose_predict;
    eskf_->ObserveSE3(pose_of_lo_, 1e-1, 1e-1);
    SE3 pose_updated = eskf_->GetNominalSE3();
    FullCloudPtr scan_pub(new FullPointCloudType);        // 放入UI
    pcl::transformPointCloud(*scan_undistort_,*scan_pub,pose_updated.matrix());
    cloud_pub_func_(cloud_pub_topic_,scan_pub,measures_.lidar_end_time_);
    pose_pub_func_(pose_pub_topic_,pose_updated,measures_.lidar_end_time_);
    frame_num_++;
}

void LooselyLIO::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

void LooselyLIO::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    sync_->ProcessCloud(msg);
}

void LooselyLIO::IMUCallBack(IMUPtr msg_in) {
    sync_->ProcessIMU(msg_in);
}

void LooselyLIO::Finish() {
    LOG(INFO) << "finish done";
}

}  // namespace wxpiggy