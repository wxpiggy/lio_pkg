#pragma once

#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

/// 部分类直接使用ch7的结果
#include "core/iekf.hpp"
#include "core/static_imu_init.h"
#include "map/ndt_inc.h"

#include "map/voxel_map.h"
#include "preprocess/measure_sync.h"

// #include "tools/ui/pangolin_window.h"

namespace wxpiggy {
namespace ui {
class PangolinWindow;
}
class VoxelLIO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct Options {
        Options() {}
        bool save_motion_undistortion_pcd_ = false;  // 是否保存去畸变前后的点云
        bool with_ui_ = true;                        // 是否带着UI
    };

    VoxelLIO(Options options = Options());
    ~VoxelLIO() = default;

    /// init without ros
    bool Init(const std::string& config_yaml);

    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);

    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);

    /// 结束程序，退出UI
    void Finish();

    /// 获取当前姿态
    NavStated GetCurrentState() const { return ieskf_.GetNominalState(); }

    /// 获取当前扫描
    CloudPtr GetCurrentScan() const { return current_scan_; }

   private:
    bool LoadFromYAML(const std::string& yaml_file);

    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup& meas);

    /// 尝试让IMU初始化
    void TryInitIMU();

    /// 利用IMU预测状态信息
    /// 这段时间的预测数据会放入imu_states_里
    void Predict();

    /// 对measures_中的点云去畸变
    void Undistort();

    /// 执行一次配准和观测
    void Align();

    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;
    StaticIMUInit imu_init_;

    /// point clouds data
    FullCloudPtr scan_undistort_{new FullPointCloudType()};  // scan after undistortion
    CloudPtr current_scan_ = nullptr;

    /// NDT数据
    IncNdt3d ndt_;
    VoxelMapManager voxel_map_;
    SE3 last_pose_;

    // flags
    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan
    std::vector<NavStated> imu_states_;
    IESKFD ieskf_;  // IESKF
    SE3 TIL_;       // Lidar与IMU之间外参

    Options options_;
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace wxpiggy
