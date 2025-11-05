#include "p2pl_icp.h"

#include <glog/logging.h>

#include <algorithm>  // std::for_each
#include <execution>  // std::execution::par, std::execution::par_unseq
#include <set>
#include <yaml-cpp/yaml.h>

#include "tools/math_utils.h"
namespace wxpiggy {

void IncIcp3d::Init(){
    // auto yaml = YAML::LoadFile(config_file);
    // auto reg = yaml["registration"];

    // options_.max_iteration_ = reg["max_iteration"].as<int>();
    // options_.voxel_size_ = reg["voxel_size"].as<double>();
    // options_.inv_voxel_size_ = 1 / options_.voxel_size_;
    // options_.min_effective_pts_ = reg["min_effective_pts"].as<int>();
    // // options_.min_pts_in_voxel_ = reg["min_pts_in_voxel"].as<int>();
    // options_.max_points_ = reg["max_pts_in_voxel"].as<int>();
    // options_.eps_ = reg["eps"].as<double>();
    // // options_.res_outlier_th_ = reg["res_outlier_th"].as<double>();
    // options_.capacity_ = reg["capacity"].as<int>();
    // options_.nearby_type_ = NearbyType(reg["nearby_type"].as<int>());
    // GenerateNearbyGrids();
}


// =======================================================
// 添加点云
// =======================================================

void IncIcp3d::AddCloud(const std::initializer_list<CloudPtr>& cloud) {
    
    flag_first_scan_ = false;
}
// =======================================================
// ICP 配准
// =======================================================

bool IncIcp3d::Align(SE3& init_pose) {
    SE3 pose = init_pose;
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
            if (nn.size() > 3) {
                Vec4d n;
                if (!wxpiggy::math::FitPlane(nn, n)) {
                    // 失败的不要
                    effect_pts[idx] = false;
                    return;
                }
                double dis = n.head<3>().dot(qs) + n[3];
                if (fabs(dis) > 0.1) {
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
        // double contidtion = computeConditionNumber(H);
        // LOG(INFO) << "contionNumber " << contidtion;
        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    init_pose = pose;
    return true;
}
double IncIcp3d::computeConditionNumber(const Eigen::MatrixXd& H) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H);
    auto eigvals = eig.eigenvalues();
    double lambda_min = eigvals.minCoeff();
    double lambda_max = eigvals.maxCoeff();
    return lambda_max / lambda_min;
}
// =======================================================
// 计算残差和雅可比
// =======================================================
void IncIcp3d::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
    // assert(target_ != nullptr && source_ != nullptr);

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
            if (fabs(dis) > 0.01) {
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

    const double R_inv =1000;

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) {
            continue;
        }

        total_res += errors[idx] * errors[idx];
        effective_num++;

        HTVH += jacobians[idx].transpose() * jacobians[idx] * R_inv;
        HTVr += -jacobians[idx].transpose() * errors[idx] * R_inv;
    }

    LOG(INFO) << "effective: " << effective_num;
}
}  // namespace wxpiggy