#pragma once
#include "voxel_map.h"
namespace wxpiggy {
void VoxelMapManager::BuildVoxelMap() {
}

// Eigen::Vector3d RGBFromVoxel(const V3D &input_point);

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithCov> &input_points) {
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithCov> &pv_list, std::vector<PointToPlane> &ptpl_list) {
}

void VoxelMapManager::build_single_residual(pointWithCov &pv,
                                            const VoxelOctoTree *current_octo,
                                            const int current_layer,
                                            bool &is_sucess,
                                            double &prob,
                                            PointToPlane &single_ptpl) {
}

// void pubVoxelMap();

// void mapSliding();
// void clearMemOutOfMap(const int &x_max,
//                       const int &x_min,
//                       const int &y_max,
//                       const int &y_min,
//                       const int &z_max,
//                       const int &z_min);

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo,
                                     const int pub_max_voxel_layer,
                                     std::vector<VoxelPlane> &plane_list) {
}

// void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub,
//                     const std::string plane_ns,
//                     const VoxelPlane &single_plane,
//                     const float alpha,
//                     const Eigen::Vector3d rgb);
void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec,
                                      const Eigen::Vector3d &y_vec,
                                      const Eigen::Vector3d &z_vec,
                                      geometry_msgs::Quaternion &q) {
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b) {
}
}  // namespace wxpiggy