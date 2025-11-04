#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

class Config {
public:
    // 获取单例实例
    static Config& GetInstance();
    
    // 加载配置文件
    bool LoadConfig(const std::string& config_file);
    struct SystemConfig{
        std::string lidar_topic;
        std::string imu_topic;
    };
    // Preprocess 配置
    struct PreprocessConfig {
        int lidar_type = 2;
        int scan_line = 32;
        double time_scale = 1e-3;
        int point_filter_num = 5;
    };
    
    // Mapping 配置
    struct MappingConfig {
        std::vector<double> extrinsic_T = {0, 0, 0.28};
        std::vector<double> extrinsic_R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        int registration_type = 0;
        
        Eigen::Vector3d GetExtrinsicTranslation() const {
            return Eigen::Vector3d(extrinsic_T[0], extrinsic_T[1], extrinsic_T[2]);
        }
        
        Eigen::Matrix3d GetExtrinsicRotation() const {
            Eigen::Matrix3d rot;
            rot << extrinsic_R[0], extrinsic_R[1], extrinsic_R[2],
                   extrinsic_R[3], extrinsic_R[4], extrinsic_R[5],
                   extrinsic_R[6], extrinsic_R[7], extrinsic_R[8];
            return rot;
        }
    };
    
    // Registration 配置
    struct RegistrationConfig {
        int max_iteration = 15;
        double voxel_size = 1.0;
        int min_effective_pts = 10;
        int min_pts_in_voxel = 5;
        int max_pts_in_voxel = 40;
        double eps = 1e-2;
        double res_outlier_th = 5.0;
        int capacity = 5000000;
        int nearby_type = 6;
    };
    
    // Init 配置
    struct InitConfig {
        double init_time_seconds = 7;
        int init_imu_queue_max_size = 2000;
        int static_odom_pulse = 5;
        double max_static_gyro_var = 0.5;
        double max_static_acce_var = 0.05;
        double gravity_norm = 9.81;
        bool use_speed_for_static_checking = false;
    };
    
    // IMU 配置
    struct IMUConfig {
        double quit_eps = 1e-3;
        double imu_dt = 0.01;
        double gyro_var = 0.1;
        double acce_var = 0.1;
        double bias_gyro_var = 0.001;
        double bias_acce_var = 0.001;
    };
    
    // GNSS 配置
    struct GNSSConfig {
        double gnss_pos_noise = 0.1;
        double gnss_height_noise = 0.1;
        double gnss_ang_noise = 1.0;
    };
    
    // 获取各模块配置
    const SystemConfig& getSystemConfig() const{ return system_;}
    const PreprocessConfig& GetPreprocessConfig() const { return preprocess_; }
    const MappingConfig& GetMappingConfig() const { return mapping_; }
    const RegistrationConfig& GetRegistrationConfig() const { return registration_; }
    const InitConfig& GetInitConfig() const { return init_; }
    const IMUConfig& GetIMUConfig() const { return imu_; }
    const GNSSConfig& GetGNSSConfig() const { return gnss_; }
    
    // 禁止拷贝和赋值
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

private:
    Config() = default;
    ~Config() = default;
    
    // 从YAML节点加载配置
    void LoadSystemConfig(const YAML::Node& node);
    void LoadPreprocessConfig(const YAML::Node& node);
    void LoadMappingConfig(const YAML::Node& node);
    void LoadRegistrationConfig(const YAML::Node& node);
    void LoadInitConfig(const YAML::Node& node);
    void LoadIMUConfig(const YAML::Node& node);
    void LoadGNSSConfig(const YAML::Node& node);
    
private:
    SystemConfig system_;
    PreprocessConfig preprocess_;
    MappingConfig mapping_;
    RegistrationConfig registration_;
    InitConfig init_;
    IMUConfig imu_;
    GNSSConfig gnss_;
    
    bool is_loaded_ = false;
};

