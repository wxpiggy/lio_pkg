#pragma once
#include <tsl/robin_map.h>

#include <list>
#include <memory>
#include <vector>

#include "common/eigen_types.h"
#include "common/point_types.h"

namespace wxpiggy {

// 体素坐标定义
struct VoxelKey {
    VoxelKey() = default;
    VoxelKey(short x, short y, short z) : x(x), y(y), z(z) {}

    bool operator==(const VoxelKey& vox) const { return x == vox.x && y == vox.y && z == vox.z; }

    inline bool operator<(const VoxelKey& vox) const { 
        return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z); 
    }

    inline VoxelKey operator+(const VoxelKey& vox) const { 
        return VoxelKey(x + vox.x, y + vox.y, z + vox.z); 
    }

    inline static VoxelKey Coordinates(const Eigen::Vector3d& point, double voxel_size) { 
        return {short(point.x() / voxel_size), short(point.y() / voxel_size), short(point.z() / voxel_size)}; 
    }

    short x, y, z;
};

}  // namespace wxpiggy

namespace std {
template <>
struct hash<wxpiggy::VoxelKey> {
    std::size_t operator()(const wxpiggy::VoxelKey& vox) const {
        const size_t kP1 = 73856093;
        const size_t kP2 = 19349669;
        const size_t kP3 = 83492791;
        return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
    }
};
}  // namespace std

namespace wxpiggy {

// 简化的平面统计量（使用 FitPlane 计算）
struct PlaneStats {
    PlaneStats() : num_points(0), mean(Eigen::Vector3d::Zero()), normal(Eigen::Vector3d::Zero()), 
                   d(0.0), is_plane_valid(false) {}

    int num_points;              // 点数量
    Eigen::Vector3d mean;        // 均值（平面中心点）
    Eigen::Vector3d normal;      // 法向量（由 FitPlane 计算）
    double d;                    // 平面方程 d（由 FitPlane 计算）
    bool is_plane_valid;         // 平面是否有效

    /**
     * 点到平面的距离
     */
    double PointToPlaneDistance(const Eigen::Vector3d& point) const { 
        return std::abs(normal.dot(point) + d); 
    }
};

// 平面体素块
struct PlaneVoxelBlock {
    explicit PlaneVoxelBlock(int max_pts = 20) : max_points_(max_pts), need_update_(false) { 
        points_.reserve(max_pts); 
    }

    PlaneStats plane_stats;                // 平面统计量
    std::vector<Eigen::Vector3d> points_;  // 缓存的点
    int max_points_;                       // 最大点数
    bool need_update_;                     // 是否需要更新平面参数

    /**
     * 添加点到体素
     */
    bool AddPoint(const Eigen::Vector3d& point) {
        if (IsFull()) return false;
        points_.push_back(point);
        need_update_ = true;
        return true;
    }

    bool IsFull() const { return points_.size() >= static_cast<size_t>(max_points_); }
    bool IsValid() const { return plane_stats.is_plane_valid; }
    int NumPoints() const { return points_.size(); }
};

// Union-Find 数据结构用于平面合并管理
class VoxelPlaneUnionFind {
   public:
    VoxelPlaneUnionFind() = default;

    /**
     * 创建新的集合
     */
    void MakeSet(const VoxelKey& v) {
        if (parent_.find(v) == parent_.end()) {
            parent_[v] = v;
            rank_[v] = 0;
        }
    }

    /**
     * 查找根节点 (带路径压缩)
     */
    VoxelKey Find(const VoxelKey& v) {
        if (parent_.find(v) == parent_.end()) {
            MakeSet(v);
            return v;
        }

        if (!(parent_[v] == v)) {
            parent_[v] = Find(parent_[v]);  // 路径压缩
        }
        return parent_[v];
    }

    /**
     * 合并两个集合
     */
    bool Union(const VoxelKey& v1, const VoxelKey& v2) {
        VoxelKey root1 = Find(v1);
        VoxelKey root2 = Find(v2);

        if (root1 == root2) return false;  // 已经在同一集合

        // 按秩合并
        if (rank_[root1] < rank_[root2]) {
            parent_[root1] = root2;
        } else if (rank_[root1] > rank_[root2]) {
            parent_[root2] = root1;
        } else {
            parent_[root2] = root1;
            rank_[root1]++;
        }
        return true;
    }

    /**
     * 判断两个体素是否在同一集合
     */
    bool IsConnected(const VoxelKey& v1, const VoxelKey& v2) { 
        return Find(v1) == Find(v2); 
    }

    /**
     * 获取所有连通分量
     */
    std::vector<std::vector<VoxelKey>> GetConnectedComponents() {
        tsl::robin_map<VoxelKey, std::vector<VoxelKey>> groups;

        for (auto& [v, _] : parent_) {
            VoxelKey root = Find(v);
            groups[root].push_back(v);
        }

        std::vector<std::vector<VoxelKey>> components;
        components.reserve(groups.size());
        for (auto& [_, group] : groups) {
            components.push_back(std::move(group));
        }
        return components;
    }

    void Clear() {
        parent_.clear();
        rank_.clear();
    }

   private:
    tsl::robin_map<VoxelKey, VoxelKey> parent_;
    tsl::robin_map<VoxelKey, int> rank_;
};

// 平面体素地图
class PlaneVoxelHashMap {
   public:
    enum class NearbyType {
        CENTER,    // 只考虑中心
        NEARBY6,   // 上下左右前后 (6 邻居)
        NEARBY18,  // 包含 6 邻居 + 12 条棱方向 (总18个)
        NEARBY26,  // 包含 6 邻居 + 12 棱方向 + 8 个角方向 (26邻居)
    };

    struct Options {
        int max_iteration_ = 20;         // ICP 最大迭代次数
        double voxel_size_ = 0.5;        // 体素大小
        double inv_voxel_size_ = 2.0;    // 体素大小之逆
        int max_points_per_voxel_ = 20;  // 每个体素最大点数
        int min_points_for_plane_ = 5;   // 形成平面的最小点数
        size_t max_voxels_ = 500000;     // 最大体素数 (LRU)
        double eps_ = 1e-2;              // 收敛判定条件

        // 平面合并参数
        double merge_distance_th_ = 0.1;  // 合并距离阈值
        double merge_normal_th_ = 0.9;    // 合并法向量阈值 (cos值)
        bool enable_plane_merge_ = true;  // 是否启用平面合并

        // FitPlane 参数
        double fit_plane_eps_ = 1e-2;     // FitPlane 的误差阈值

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

    PlaneVoxelHashMap() : options_() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    explicit PlaneVoxelHashMap(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    /**
     * 添加点云到地图
     */
    void AddCloud(CloudPtr cloud_world);

    /**
     * 设置源点云
     */
    void SetSource(CloudPtr source) { source_ = source; }

    /**
     * 点到面 ICP 配准
     */
    bool AlignICP(SE3& init_pose);

    /**
     * 计算残差和雅可比
     */
    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr);

    /**
     * 查找最近邻平面
     */
    PlaneVoxelBlock* FindNearestPlane(const Eigen::Vector3d& point);

    /**
     * 合并相邻平面
     */
    void MergeAdjacentPlanes();

    /**
     * 获取统计信息
     */
    inline int NumVoxels() const { return voxel_map_.size(); }
    inline int NumValidPlanes() const {
        int count = 0;
        for (const auto& [_, it] : voxel_map_) {
            if (it->second.IsValid()) count++;
        }
        return count;
    }

   private:
    using VoxelData = std::pair<VoxelKey, PlaneVoxelBlock>;

    /**
     * 检查两个平面是否可以合并
     */
    bool CanMergePlanes(const PlaneVoxelBlock& p1, const PlaneVoxelBlock& p2) const;

    /**
     * 合并两个体素的平面
     */
    void MergeTwoVoxels(const VoxelKey& v1, const VoxelKey& v2);

    /**
     * 生成邻近体素偏移
     */
    void GenerateNearbyGrids();

    CloudPtr source_ = nullptr;
    Options options_;

    std::list<VoxelData> voxel_data_;                                     // LRU 链表
    tsl::robin_map<VoxelKey, std::list<VoxelData>::iterator> voxel_map_;  // 体素哈希表
    std::vector<VoxelKey> nearby_grids_;                                  // 邻近体素偏移
    VoxelPlaneUnionFind plane_merge_uf_;                                  // 平面合并管理
    bool flag_first_scan_ = true;
};

}  // namespace wxpiggy