#pragma once
#include <tsl/robin_map.h>

#include <list>

#include "common/eigen_types.h"
#include "common/point_types.h"
#include "registration/registration_base.h"
namespace wxpiggy {


struct voxel {
    voxel() = default;

    voxel(short x, short y, short z) : x(x), y(y), z(z) {}

    bool operator==(const voxel& vox) const { return x == vox.x && y == vox.y && z == vox.z; }

    inline bool operator<(const voxel& vox) const {
        return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
    }
    inline voxel operator+(const voxel& vox) const {
        voxel v;
        v.x = x + vox.x;
        v.y = y + vox.y;
        v.z = z + vox.z;
        return v;
    }
    inline static voxel coordinates(const Eigen::Vector3d& point, double voxel_size) {
        return {short(point.x() / voxel_size), short(point.y() / voxel_size), short(point.z() / voxel_size)};
    }

    short x;
    short y;
    short z;
};

struct voxelBlock {
    explicit voxelBlock(int num_points_ = 20) : num_points(num_points_) { points.reserve(num_points_); }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;

    bool IsFull() const { return num_points == points.size(); }

    void AddPoint(const Eigen::Vector3d& point) {
        assert(num_points > points.size());
        points.push_back(point);
    }

    inline int NumPoints() const { return points.size(); }

    inline int Capacity() { return num_points; }

   private:
    int num_points;
};

typedef tsl::robin_map<voxel, voxelBlock> voxelHashMap;

}  // namespace wxpiggy

namespace std {
template <>
struct hash<wxpiggy::voxel> {
    std::size_t operator()(const wxpiggy::voxel& vox) const {
        const size_t kP1 = 73856093;
        const size_t kP2 = 19349669;
        const size_t kP3 = 83492791;
        return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
    }
};
}  // namespace std
namespace wxpiggy {
class IncIcp3d  :public RegistrationBase {
   public:
    enum class NearbyType {
        CENTER = 0 ,    // 只考虑中心
        NEARBY6 = 6 ,   
        NEARBY18 = 18,   
        NEARBY26 = 26,  
    };

    struct Options {
        int max_iteration_ = 4;        // ICP 最大迭代次数
        double voxel_size_ = 0.5;      // 体素大小
        double inv_voxel_size_ = 2.0;  // 体素大小之逆
        int min_effective_pts_ = 10;   
        double eps_ = 1e-2;         // 收敛判定条件
        size_t capacity_ = 500000;  // LRU 最大容量
        size_t max_points_ = 40;
        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

    using VoxelKeyType = voxel;  
    using ValueType = voxelBlock;
    using HashMapType = voxelHashMap;

    IncIcp3d() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        // GenerateNearbyGrids();
    }

    explicit IncIcp3d(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        // GenerateNearbyGrids();
    }

    /// 获取统计信息
    inline int NumGrids() const { return grids_.size(); }

    /// 添加点云到local map
    void AddCloud(CloudPtr cloud_world) override;

    /// 设置源点云
    void SetSource(CloudPtr source) override{ source_ = source; }

    /// 点到面 ICP 配准
    bool Align(SE3& init_pose)override;
    /**
     * 计算残差和雅可比
     * @param pose 当前位姿
     * @param HTVH 累积的 Hessian
     * @param HTVr 累积的梯度
     */
    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) override ;
    bool FindKNearestNeighbors(const Eigen::Vector3d& point,
                               int k,
                               std::vector<Eigen::Vector3d>& neighbors,
                               double max_distance = 2.0);
    void LoadFromYAML(const std::string& config_file) override;    
    double computeConditionNumber(const Eigen::MatrixXd& H);
   private:
    void GenerateNearbyGrids();

    CloudPtr source_ = nullptr;
    Options options_;

    using KeyAndData = std::pair<VoxelKeyType, voxelBlock>;
    std::list<KeyAndData> data_;                                           // 缓存所有 voxel 数据
    tsl::robin_map<VoxelKeyType, std::list<KeyAndData>::iterator> grids_;  // key -> list iterator
    std::vector<VoxelKeyType> nearby_grids_;                               // 最近邻
    bool flag_first_scan_ = true;
};
}  // namespace wxpiggy