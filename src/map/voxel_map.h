#include <list>
#include <memory>

#include "common/eigen_types.h"
#include "common/point_types.h"
#include "common/voxel_key.h"
#include "tsl/robin_map.h"
namespace wxpiggy {
struct PointWithCov {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d point;  //  raw point
    Eigen::Matrix3d cov;    //  global frame
};
struct Plane {
    Eigen::Matrix3d ppt = Eigen::Matrix3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    int n;
};
struct PointToPlane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point_lidar = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_norm = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d cov_lidar = Eigen::Matrix3d::Zero();
    ;
    Eigen::Matrix3d cov_world = Eigen::Matrix3d::Zero();
    ;
    bool is_valid = false;
    double residual = 0.0;
};
class VoxelMap;

class Voxel {
   public:
    Voxel(int _max_point_thresh, int _update_point_thresh, double _plane_thresh, VoxelKey _position, VoxelMap *_map);

    void updatePlane();

    void addToPlane(const PointWithCov &pv);

    void addPoint(const PointWithCov &pv);

    void pushPoint(const PointWithCov &pv);

    void merge();

   public:
    static uint64_t count;
    int max_point_thresh;
    int update_point_thresh;
    double plane_thresh;
    bool is_init;
    bool is_plane;
    bool update_enable;
    int newly_add_point;
    bool merged;
    uint64_t group_id;
    std::vector<PointWithCov> temp_points;
    VoxelKey position;
    VoxelMap *map;
    std::shared_ptr<Plane> plane;
    Eigen::Vector3d center;
    std::list<VoxelKey>::iterator cache_it;
    static double merge_thresh_for_angle;
    static double merge_thresh_for_distance;
};

// typedef std::unordered_map<VoxelKey, std::shared_ptr<VoxelGrid>, VoxelKey::Hasher> Featmap;
typedef tsl::robin_map<VoxelKey, Voxel> VoxelHashMap;
class VoxelMap {
   public:
    VoxelMap(int _max_point_thresh,
             int _update_point_thresh,
             double _plane_thresh,
             double _voxel_size,
             int capacity = 2000000);

    VoxelKey index(const Eigen::Vector3d &point);

    void build(std::vector<PointWithCov> &pvs);

    void update(std::vector<PointWithCov> &pvs);

    bool buildResidual(PointToPlane &data, std::shared_ptr<Voxel> voxel);

   public:
    int max_point_thresh;
    int update_point_thresh;
    double plane_thresh;
    double voxel_size;
    VoxelHashMap featmap;
    std::list<VoxelKey> cache;
    int capacity;
    CloudPtr source_ = nullptr;
};

}  // namespace wxpiggy