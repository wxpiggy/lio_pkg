#include "icp_inc.h"

#include <glog/logging.h>

#include <algorithm>  // std::for_each
#include <execution>  // std::execution::par, std::execution::par_unseq
#include <set>

#include "common/math_utils.h"
namespace wxpiggy {
void IncIcp3d::GenerateNearbyGrids() {
    nearby_grids_.clear();

    auto addVoxelOffset = [&](int dx, int dy, int dz) { nearby_grids_.emplace_back(VoxelKeyType(dx, dy, dz)); };

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
// 添加点云
// =======================================================

void IncIcp3d::AddCloud(CloudPtr cloud_world) {
    // 完全按照NDT的LRU逻辑实现
    for (const auto& p : cloud_world->points) {
        Vec3d pt = ToVec3d(p);
        VoxelKeyType key = voxel::coordinates(pt, options_.voxel_size_);

        auto iter = grids_.find(key);
        if (iter == grids_.end()) {
            // voxel 不存在，创建新体素
            voxelBlock block;
            block.AddPoint(pt);
            data_.emplace_front(key, std::move(block));  // 插入到队列头部
            grids_.emplace(key, data_.begin());

            // 立即检查容量，执行LRU删除
            if (data_.size() >= options_.capacity_) {
                grids_.erase(data_.back().first);  // 删除哈希表映射
                data_.pop_back();                  // 删除队列尾部
            }
        } else {
            // voxel 已存在
            auto& voxel_block = iter.value()->second;
            if (voxel_block.NumPoints() < options_.max_points_) {
                voxel_block.AddPoint(pt);  // 添加点
            }
            // 更新LRU：将使用过的体素移到队列头部
            data_.splice(data_.begin(), data_, iter.value());
            iter.value() = data_.begin();
        }
    }

    flag_first_scan_ = false;
}
bool IncIcp3d::FindKNearestNeighbors(const Eigen::Vector3d& point, int k, std::vector<Eigen::Vector3d>& neighbors, double max_distance) {
    neighbors.clear();
    if (grids_.empty() || k <= 0) return false;

    voxel center_voxel = voxel::coordinates(point, options_.voxel_size_);

    // 临时存储候选点和距离
    std::vector<std::pair<double, Eigen::Vector3d>> candidate_pts;

    for (const auto& offset : nearby_grids_) {
        voxel neighbor_voxel = center_voxel + offset;
        auto it = grids_.find(neighbor_voxel);
        if (it != grids_.end()) {
            voxelBlock& block = it->second->second;
            for (const auto& pt : block.points) {
                double dist2 = (point - pt).squaredNorm();
                if (dist2 <= max_distance * max_distance) {
                    candidate_pts.emplace_back(dist2, pt);
                }
            }
        }
    }

    if (candidate_pts.size() < k) return false;

    // 按距离排序
    std::sort(candidate_pts.begin(), candidate_pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    // 取前 k 个最近邻
    int n = std::min(k, static_cast<int>(candidate_pts.size()));
    neighbors.reserve(n);
    for (int i = 0; i < n; ++i) {
        neighbors.push_back(candidate_pts[i].second);
    }

    return true;
}

// =======================================================
// ICP 配准
// =======================================================

bool IncIcp3d::AlignICP(SE3& init_pose) {
    LOG(INFO) << "aligning with point to plane";
    assert(target_ != nullptr && source_ != nullptr);
    // 整体流程与p2p一致，读者请关注变化部分

    SE3 pose = init_pose;
    // if (!options_.use_initial_translation_) {
    //     pose.translation() = target_center_ - source_center_;  // 设置平移初始值
    // }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
    std::vector<double> errors(index.size());

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // gauss-newton 迭代
        // 最近邻，可以并发
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的q
            std::vector<Eigen::Vector3d> nn;
            FindKNearestNeighbors(qs, 5, nn);
            // GetClosestPoint(ToPointType(qs), nn, 5);  // 这里取5个最近邻
            if (nn.size() >= 5) {
                // convert to eigen
                // std::vector<Vec3d> nn_eigen;
                // for (int i = 0; i < nn.size(); ++i) {
                //     nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                // }

                Vec4d n;
                if (!wxpiggy::math::FitPlane(nn, n)) {
                    // 失败的不要
                    effect_pts[idx] = false;
                    return;
                }

                double dis = n.head<3>().dot(qs) + n[3];
                if (fabs(dis) > 0.5) {
                    // 点离的太远了不要
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;

                // build residual
                Eigen::Matrix<double, 1, 6> J;
                J.block<1, 3>(0, 0) = -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(q);
                J.block<1, 3>(0, 3) = n.head<3>().transpose();

                jacobians[idx] = J;
                errors[idx] = dis;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(index.begin(),
                                         index.end(),
                                         std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
                                         [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                                             if (!effect_pts[idx]) {
                                                 return pre;
                                             } else {
                                                 total_res += errors[idx] * errors[idx];
                                                 effective_num++;
                                                 return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx], pre.second - jacobians[idx].transpose() * errors[idx]);
                                             }
                                         });

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        // if (gt_set_) {
        //     double pose_error = (gt_pose_.inverse() * pose).log().norm();
        //     LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        // }

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
void IncIcp3d::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
    assert(target_ != nullptr && source_ != nullptr);

    SE3 pose = input_pose;

    // 对点的索引，预先生成
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    int total_size = index.size();

    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 1, 18>> jacobians(total_size);
    std::vector<double> errors(total_size);

    // 最近邻，可以并发
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q;  // 转换之后的q

        std::vector<Eigen::Vector3d> nn;
        FindKNearestNeighbors(qs, 5, nn);

        if (nn.size() > 3) {
            Vec4d n;
            if (!wxpiggy::math::FitPlane(nn, n)) {
                effect_pts[idx] = false;
                return;
            }

            double dis = n.head<3>().dot(qs) + n[3];
            if (fabs(dis) > 5) {
                effect_pts[idx] = false;
                return;
            }

            effect_pts[idx] = true;

            // build residual - 18维雅可比
            Eigen::Matrix<double, 1, 18> J;
            J.setZero();
            J.block<1, 3>(0, 0) = n.head<3>().transpose();                                       // 对p
            J.block<1, 3>(0, 6) = -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(q);  // 对R

            jacobians[idx] = J;
            errors[idx] = dis;
        } else {
            effect_pts[idx] = false;
        }
    });

    // 累加Hessian和error
    double total_res = 0;
    int effective_num = 0;

    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 1000;  // 每个点反馈的info因子

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
}  // namespace wxpiggy
