#include "voxel_map.h"

namespace wxpiggy {

// ==================== Voxel ====================
uint64_t Voxel::count = 0;
double Voxel::merge_thresh_for_angle = 0.1;
double Voxel::merge_thresh_for_distance = 0.04;

Voxel::Voxel(int _max_point_thresh, int _update_point_thresh, double _plane_thresh, VoxelKey _position, VoxelMap *_map) 
    : max_point_thresh(_max_point_thresh),
      update_point_thresh(_update_point_thresh),
      plane_thresh(_plane_thresh),
      position(_position),
      map(_map),
      is_init(false),
      is_plane(false),
      update_enable(false),
      newly_add_point(0),
      merged(false),
      group_id(0),
      center(Eigen::Vector3d::Zero())
{
    
}

void Voxel::updatePlane() {
   
}

void Voxel::addToPlane(const PointWithCov &pv) {
    
}

void Voxel::addPoint(const PointWithCov &pv) {
    
}

void Voxel::pushPoint(const PointWithCov &pv) {
    
}

void Voxel::merge() {
    
}

// ==================== VoxelMap ====================
VoxelMap::VoxelMap(int _max_point_thresh,
                   int _update_point_thresh,
                   double _plane_thresh,
                   double _voxel_size,
                   int capacity_)
    : max_point_thresh(_max_point_thresh),
      update_point_thresh(_update_point_thresh),
      plane_thresh(_plane_thresh),
      voxel_size(_voxel_size),
      capacity(capacity_)
{
    // constructor body left empty
}

VoxelKey VoxelMap::index(const Eigen::Vector3d &point) {
    // empty
    return VoxelKey();
}

void VoxelMap::build(std::vector<PointWithCov> &pvs) {
    // empty
}

void VoxelMap::update(std::vector<PointWithCov> &pvs) {
    // empty
}

bool VoxelMap::buildResidual(PointToPlane &data, std::shared_ptr<Voxel> voxel) {
    // empty
    return false;
}

}  // namespace wxpiggy