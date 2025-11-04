#include "config.h"
#include <iostream>

Config& Config::GetInstance() {
    static Config instance;
    return instance;
}

bool Config::LoadConfig(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        if(config["system"]){
            LoadSystemConfig(config["system"]);
        }
        // 加载各模块配置
        if (config["preprocess"]) {
            LoadPreprocessConfig(config["preprocess"]);
        }
        
        if (config["mapping"]) {
            LoadMappingConfig(config["mapping"]);
        }
        
        if (config["registration"]) {
            LoadRegistrationConfig(config["registration"]);
        }
        
        if (config["init"]) {
            LoadInitConfig(config["init"]);
        }
        
        if (config["imu"]) {
            LoadIMUConfig(config["imu"]);
        }
        
        if (config["gnss"]) {
            LoadGNSSConfig(config["gnss"]);
        }
        
        is_loaded_ = true;
        std::cout << "Config loaded successfully from: " << config_file << std::endl;
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load config file: " << config_file 
                  << ", error: " << e.what() << std::endl;
        return false;
    }
}
void Config::LoadSystemConfig(const YAML::Node& node){
    if(node["lidar_topic"]) system_.lidar_topic = node["lidar_topic"].as<std::string>();
    if(node["imu_topic"]) system_.imu_topic = node["imu_topic"].as<std::string>();
}
void Config::LoadPreprocessConfig(const YAML::Node& node) {
    if (node["lidar_type"]) preprocess_.lidar_type = node["lidar_type"].as<int>();
    if (node["scan_line"]) preprocess_.scan_line = node["scan_line"].as<int>();
    if (node["time_scale"]) preprocess_.time_scale = node["time_scale"].as<double>();
    if (node["point_filter_num"]) preprocess_.point_filter_num = node["point_filter_num"].as<int>();
}

void Config::LoadMappingConfig(const YAML::Node& node) {
    if (node["extrinsic_T"]) mapping_.extrinsic_T = node["extrinsic_T"].as<std::vector<double>>();
    if (node["extrinsic_R"]) mapping_.extrinsic_R = node["extrinsic_R"].as<std::vector<double>>();
    if (node["registration_type"]) mapping_.registration_type = node["registration_type"].as<int>();
}

void Config::LoadRegistrationConfig(const YAML::Node& node) {
    if (node["max_iteration"]) registration_.max_iteration = node["max_iteration"].as<int>();
    if (node["voxel_size"]) registration_.voxel_size = node["voxel_size"].as<double>();
    if (node["min_effective_pts"]) registration_.min_effective_pts = node["min_effective_pts"].as<int>();
    if (node["min_pts_in_voxel"]) registration_.min_pts_in_voxel = node["min_pts_in_voxel"].as<int>();
    if (node["max_pts_in_voxel"]) registration_.max_pts_in_voxel = node["max_pts_in_voxel"].as<int>();
    if (node["eps"]) registration_.eps = node["eps"].as<double>();
    if (node["res_outlier_th"]) registration_.res_outlier_th = node["res_outlier_th"].as<double>();
    if (node["capacity"]) registration_.capacity = node["capacity"].as<int>();
    if (node["nearby_type"]) registration_.nearby_type = node["nearby_type"].as<int>();
}

void Config::LoadInitConfig(const YAML::Node& node) {
    if (node["init_time_seconds"]) init_.init_time_seconds = node["init_time_seconds"].as<double>();
    if (node["init_imu_queue_max_size"]) init_.init_imu_queue_max_size = node["init_imu_queue_max_size"].as<int>();
    if (node["static_odom_pulse"]) init_.static_odom_pulse = node["static_odom_pulse"].as<int>();
    if (node["max_static_gyro_var"]) init_.max_static_gyro_var = node["max_static_gyro_var"].as<double>();
    if (node["max_static_acce_var"]) init_.max_static_acce_var = node["max_static_acce_var"].as<double>();
    if (node["gravity_norm"]) init_.gravity_norm = node["gravity_norm"].as<double>();
    if (node["use_speed_for_static_checking"]) init_.use_speed_for_static_checking = node["use_speed_for_static_checking"].as<bool>();
}

void Config::LoadIMUConfig(const YAML::Node& node) {
    if (node["quit_eps"]) imu_.quit_eps = node["quit_eps"].as<double>();
    if (node["imu_dt"]) imu_.imu_dt = node["imu_dt"].as<double>();
    if (node["gyro_var"]) imu_.gyro_var = node["gyro_var"].as<double>();
    if (node["acce_var"]) imu_.acce_var = node["acce_var"].as<double>();
    if (node["bias_gyro_var"]) imu_.bias_gyro_var = node["bias_gyro_var"].as<double>();
    if (node["bias_acce_var"]) imu_.bias_acce_var = node["bias_acce_var"].as<double>();
}

void Config::LoadGNSSConfig(const YAML::Node& node) {
    if (node["gnss_pos_noise"]) gnss_.gnss_pos_noise = node["gnss_pos_noise"].as<double>();
    if (node["gnss_height_noise"]) gnss_.gnss_height_noise = node["gnss_height_noise"].as<double>();
    if (node["gnss_ang_noise"]) gnss_.gnss_ang_noise = node["gnss_ang_noise"].as<double>();
}