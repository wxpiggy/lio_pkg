#include "voxel_map.h"

#include <glog/logging.h>

#include <algorithm>
#include <execution>
#include <set>

#include "common/math_utils.h"

namespace wxpiggy {

// =======================================================
// 生成邻近体素偏移
// =======================================================
void PlaneVoxelHashMap::GenerateNearbyGrids() {
    nearby_grids_.clear();

    auto addVoxelOffset = [&](int dx, int dy, int dz) { 
        nearby_grids_.emplace_back(VoxelKey(dx, dy, dz)); 
    };

    switch (options_.nearby_type_) {
        case NearbyType::CENTER:
            addVoxelOffset(0, 0, 0);
            break;

        case NearbyType::NEARBY6:
            addVoxelOffset(0, 0, 0);
            addVoxelOffset(-1, 0, 0);
            addVoxelOffset(1, 0, 0);
            addVoxelOffset(0, -1, 0);
            addVoxelOffset(0, 1, 0);
            addVoxelOffset(0, 0, -1);
            addVoxelOffset(0, 0, 1);
            break;

        case NearbyType::NEARBY18:
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        int sum = std::abs(dx) + std::abs(dy) + std::abs(dz);
                        if (sum >= 1 && sum <= 2) addVoxelOffset(dx, dy, dz);
                    }
            break;

        case NearbyType::NEARBY26:
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                        if (dx != 0 || dy != 0 || dz != 0) addVoxelOffset(dx, dy, dz);
            break;
    }
}

// =======================================================
// 添加点云到地图
// =======================================================
void PlaneVoxelHashMap::AddCloud(CloudPtr cloud_world) {
    // 按照 LRU 逻辑实现
    for (const auto& p : cloud_world->points) {
        Vec3d pt = ToVec3d(p);
        VoxelKey key = VoxelKey::Coordinates(pt, options_.voxel_size_);

        auto iter = voxel_map_.find(key);
        if (iter == voxel_map_.end()) {
            // voxel 不存在，创建新体素
            PlaneVoxelBlock block(options_.max_points_per_voxel_);
            block.AddPoint(pt);
            voxel_data_.push_front({key, std::move(block)});
            voxel_map_.insert({key, voxel_data_.begin()});
            
            // LRU 淘汰
            if (voxel_data_.size() > options_.max_voxels_) {
                voxel_map_.erase(voxel_data_.back().first);
                voxel_data_.pop_back();
            }
        } else {
            // voxel 已存在
            auto& voxel_block = iter->second->second;
            if (!voxel_block.IsFull()) {
                voxel_block.AddPoint(pt);
            }
            // LRU 更新：移到队列头部
            voxel_data_.splice(voxel_data_.begin(), voxel_data_, iter->second);
            iter.value() = voxel_data_.begin();
        }
    }

    // 批量更新平面参数 - 使用 FitPlane
    for (auto& [key, block] : voxel_data_) {
        if (block.points_.size() >= options_.min_points_for_plane_ && block.need_update_) {
            // 使用 FitPlane 拟合平面
            Vec4d plane_coeffs;
            if (wxpiggy::math::FitPlane(block.points_, plane_coeffs, 1e-2)) {
                // 拟合成功，更新平面参数
                block.plane_stats.normal = plane_coeffs.head<3>();
                block.plane_stats.d = plane_coeffs[3];
                block.plane_stats.is_plane_valid = true;
                block.plane_stats.num_points = block.points_.size();
                
                // 计算均值
                block.plane_stats.mean = Eigen::Vector3d::Zero();
                for (const auto& pt : block.points_) {
                    block.plane_stats.mean += pt;
                }
                block.plane_stats.mean /= block.points_.size();
                
                block.need_update_ = false;
            } else {
                block.plane_stats.is_plane_valid = false;
            }
        }
    }

    // 可选：执行平面合并
    // if (options_.enable_plane_merge_) {
    //     MergeAdjacentPlanes();
    // }

    LOG(INFO) << "cloud size: " << voxel_data_.size() 
              << ", valid planes: " << NumValidPlanes();
    flag_first_scan_ = false;
}

// =======================================================
// 查找最近邻平面
// =======================================================
PlaneVoxelBlock* PlaneVoxelHashMap::FindNearestPlane(const Eigen::Vector3d& point) {
    VoxelKey center_voxel = VoxelKey::Coordinates(point, options_.voxel_size_);

    PlaneVoxelBlock* nearest_plane = nullptr;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& offset : nearby_grids_) {
        VoxelKey neighbor_voxel = center_voxel + offset;
        auto it = voxel_map_.find(neighbor_voxel);
        
        if (it != voxel_map_.end()) {
            PlaneVoxelBlock& block = it->second->second;
            if (block.IsValid()) {
                double dist = block.plane_stats.PointToPlaneDistance(point);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_plane = &block;
                }
            }
        }
    }

    return nearest_plane;
}

// =======================================================
// 点到面 ICP 配准
// =======================================================
bool PlaneVoxelHashMap::AlignICP(SE3& init_pose) {
    LOG(INFO) << "aligning with plane to plane ICP";
    assert(source_ != nullptr);

    SE3 pose = init_pose;

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
    std::vector<double> errors(index.size());

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // 最近邻搜索，可以并发
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的 q

            // 查找邻近体素并收集点
            VoxelKey center_voxel = VoxelKey::Coordinates(qs, options_.voxel_size_);
            std::vector<Vec3d> nearby_points;

            for (const auto& offset : nearby_grids_) {
                VoxelKey neighbor_voxel = center_voxel + offset;
                auto it = voxel_map_.find(neighbor_voxel);
                
                if (it != voxel_map_.end()) {
                    PlaneVoxelBlock& block = it->second->second;
                    for (const auto& pt : block.points_) {
                        nearby_points.push_back(pt);
                        if (nearby_points.size() >= 20) break;  // 最多取20个点
                    }
                }
                if (nearby_points.size() >= 20) break;
            }

            if (nearby_points.size() >= 5) {
                // 使用 FitPlane 拟合平面
                Vec4d plane_coeffs;
                if (!wxpiggy::math::FitPlane(nearby_points, plane_coeffs, 1e-2)) {
                    effect_pts[idx] = false;
                    return;
                }

                Vec3d n = plane_coeffs.head<3>();
                double d = plane_coeffs[3];
                double dis = n.dot(qs) + d;
                
                if (fabs(dis) > 0.5) {
                    // 点离平面太远，不要
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;

                // 构建残差
                Eigen::Matrix<double, 1, 6> J;
                J.block<1, 3>(0, 0) = -n.transpose() * pose.so3().matrix() * SO3::hat(q);
                J.block<1, 3>(0, 3) = n.transpose();

                jacobians[idx] = J;
                errors[idx] = dis;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 累加 Hessian 和 error
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(
            index.begin(),
            index.end(),
            std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](
                const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;
                } else {
                    total_res += errors[idx] * errors[idx];
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(
                        pre.first + jacobians[idx].transpose() * jacobians[idx],
                        pre.second - jacobians[idx].transpose() * errors[idx]);
                }
            });

        if (effective_num < options_.min_points_for_plane_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        LOG(INFO) << "iter " << iter << " total res: " << total_res 
                  << ", eff: " << effective_num 
                  << ", mean res: " << total_res / effective_num 
                  << ", dxn: " << dx.norm();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    init_pose = pose;
    return true;
}

// =======================================================
// 计算残差和雅可比
// =======================================================
void PlaneVoxelHashMap::ComputeResidualAndJacobians(const SE3& input_pose, 
                                                     Mat18d& HTVH, 
                                                     Vec18d& HTVr) {
    assert(source_ != nullptr);

    SE3 pose = input_pose;

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    int total_size = index.size();

    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 1, 18>> jacobians(total_size);
    std::vector<double> errors(total_size);

    // 最近邻搜索，可以并发
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q;  // 转换之后的 q

        // 查找邻近体素并收集点
        VoxelKey center_voxel = VoxelKey::Coordinates(qs, options_.voxel_size_);
        std::vector<Vec3d> nearby_points;

        for (const auto& offset : nearby_grids_) {
            VoxelKey neighbor_voxel = center_voxel + offset;
            auto it = voxel_map_.find(neighbor_voxel);
            
            if (it != voxel_map_.end()) {
                PlaneVoxelBlock& block = it->second->second;
                for (const auto& pt : block.points_) {
                    nearby_points.push_back(pt);
                    if (nearby_points.size() >= 20) break;
                }
            }
            if (nearby_points.size() >= 20) break;
        }

        if (nearby_points.size() >= 5) {
            // 使用 FitPlane 拟合平面
            Vec4d plane_coeffs;
            if (!wxpiggy::math::FitPlane(nearby_points, plane_coeffs, 1e-2)) {
                effect_pts[idx] = false;
                return;
            }

            Vec3d n = plane_coeffs.head<3>();
            double d = plane_coeffs[3];
            double dis = n.dot(qs) + d;
            
            if (fabs(dis) > 5.0) {
                effect_pts[idx] = false;
                return;
            }

            effect_pts[idx] = true;

            // 构建 18 维雅可比
            Eigen::Matrix<double, 1, 18> J;
            J.setZero();
            J.block<1, 3>(0, 0) = n.transpose();  // 对 p
            J.block<1, 3>(0, 6) = -n.transpose() * pose.so3().matrix() * SO3::hat(q);  // 对 R

            jacobians[idx] = J;
            errors[idx] = dis;
        } else {
            effect_pts[idx] = false;
        }
    });

    // 累加 Hessian 和 error
    double total_res = 0;
    int effective_num = 0;

    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 1000;  // 每个点反馈的 info 因子

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) {
            continue;
        }

        total_res += errors[idx] * errors[idx];
        effective_num++;

        HTVH += jacobians[idx].transpose() * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * errors[idx] * info_ratio;
    }

    LOG(INFO) << "effective: " << effective_num;
}

// =======================================================
// 平面合并相关
// =======================================================
bool PlaneVoxelHashMap::CanMergePlanes(const PlaneVoxelBlock& p1, 
                                        const PlaneVoxelBlock& p2) const {
    if (!p1.IsValid() || !p2.IsValid()) return false;

    // 法向量相似性检查
    double cos_angle = p1.plane_stats.normal.dot(p2.plane_stats.normal);
    if (cos_angle < options_.merge_normal_th_) return false;

    // 距离检查：计算一个平面中心点到另一个平面的距离
    double dist1 = p2.plane_stats.PointToPlaneDistance(p1.plane_stats.mean);
    double dist2 = p1.plane_stats.PointToPlaneDistance(p2.plane_stats.mean);

    return (dist1 < options_.merge_distance_th_) && (dist2 < options_.merge_distance_th_);
}

// void PlaneVoxelHashMap::MergeTwoVoxels(const VoxelKey& v1, const VoxelKey& v2) {
//     auto it1 = voxel_map_.find(v1);
//     auto it2 = voxel_map_.find(v2);

//     if (it1 == voxel_map_.end() || it2 == voxel_map_.end()) return;

//     PlaneVoxelBlock& block1 = it1->second->second;
//     PlaneVoxelBlock& block2 = it2->second->second;

//     // 合并平面统计量
//     block1.plane_stats.MergeWith(block2.plane_stats);
//     block1.UpdatePlane();
// }

// void PlaneVoxelHashMap::MergeAdjacentPlanes() {
//     if (!options_.enable_plane_merge_) return;

//     plane_merge_uf_.Clear();

//     // 初始化 Union-Find
//     for (const auto& [key, block] : voxel_data_) {
//         if (block.IsValid()) {
//             plane_merge_uf_.MakeSet(key);
//         }
//     }

//     // 检查相邻体素是否可以合并
//     for (const auto& [key, block] : voxel_data_) {
//         if (!block.IsValid()) continue;

//         for (const auto& offset : nearby_grids_) {
//             if (offset.x == 0 && offset.y == 0 && offset.z == 0) continue;

//             VoxelKey neighbor_key = key + offset;
//             auto it = voxel_map_.find(neighbor_key);
            
//             if (it != voxel_map_.end()) {
//                 PlaneVoxelBlock& neighbor_block = it->second->second;
//                 if (CanMergePlanes(block, neighbor_block)) {
//                     plane_merge_uf_.Union(key, neighbor_key);
//                 }
//             }
//         }
//     }

//     // 获取连通分量并合并
//     auto components = plane_merge_uf_.GetConnectedComponents();
    
//     for (const auto& component : components) {
//         if (component.size() <= 1) continue;

//         // 选择第一个体素作为代表，合并其他体素到它
//         const VoxelKey& root = component[0];
//         for (size_t i = 1; i < component.size(); ++i) {
//             MergeTwoVoxels(root, component[i]);
//         }
//     }

//     LOG(INFO) << "merged " << components.size() << " plane groups";
// }

}  // namespace wxpiggy