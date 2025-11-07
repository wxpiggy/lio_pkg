#include "p2pl_icp.h"

#include <glog/logging.h>

#include <algorithm>  // std::for_each
#include <execution>  // std::execution::par, std::execution::par_unseq
#include <set>
#include <yaml-cpp/yaml.h>

#include "tools/math_utils.h"
namespace wxpiggy {

void IncIcp3d::Init(){
    std::cout << "ivox init begin" << std::endl;
    IVoxType::Options options;
    // options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    // options.capacity_ = 100000;
    options.resolution_ = 0.5;
    // options.inv_resolution_ = 2.0; 
    ivox_ = std::make_shared<IVoxType>(options);
    std::cout << "ivox init" << std::endl;
}


// =======================================================
// 添加点云
// =======================================================

void IncIcp3d::AddCloud(const std::initializer_list<CloudPtr>& cloud) {
    auto cloud_world = *cloud.begin();
    
    if(!flag_first_scan_){
        ivox_->AddPoints(cloud_world->points);
        flag_first_scan_ = true;
        return;
    }
    
    IVoxType::PointVector points_to_add;
    IVoxType::PointVector point_no_need_downsample;
    int size = cloud_world->size();
    points_to_add.reserve(cloud_world->size());
    point_no_need_downsample.reserve(cloud_world->size());
    std::vector<size_t> index(size);
    for (size_t i = 0; i < size; ++i) {
        index[i] = i;
    }
    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i){
        auto point_world = cloud_world->points[i];
        if(!nearest_points_[i].empty()){
            
            const IVoxType::PointVector &points_near = nearest_points_[i];
            Eigen::Matrix<float, 3, 1> center =
                    ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() +
                        0.5) * filter_size_map_min_;
            Eigen::Matrix<float, 3, 1> dis_2_center = points_near[0].getVector3fMap() - center;
            if (std::fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ && std::fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ && std::fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }
            bool need_add = true;
            float dist = (point_world.getVector3fMap() - center).squaredNorm();
            if (points_near.size() >= 5u) {
                for (int readd_i = 0; readd_i < 5; readd_i++) {
                    if ((points_near[readd_i].getVector3fMap() - center).squaredNorm() < dist + 1.0e-6) {
                            need_add = false;
                            break;
                        }
                    }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        }
        else {
            points_to_add.emplace_back(point_world);
        }
    });

    ivox_->AddPoints(points_to_add);
    ivox_->AddPoints(point_no_need_downsample);
    
}
// =======================================================
// ICP 配准
// =======================================================


bool IncIcp3d::Align(SE3& init_pose) {
    std::cout << ivox_->NumValidGrids()<< std::endl;
    SE3 pose = init_pose;
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
    std::vector<double> errors(index.size());
    nearest_points_.clear();
    nearest_points_.resize(index.size());
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // gauss-newton 迭代
        // 最近邻，可以并发
        
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的q
            Point p;
            p.x = qs.x();
            p.y = qs.y();
            p.z = qs.z();
            // std::vector<Eigen::Vector3d> nn;
            auto &points_near = nearest_points_[idx];
            
            ivox_->GetClosestPoint(p,points_near,5);
            if (points_near.size() >= 4) {
                std::vector<Eigen::Vector3d> nn;
                nn.reserve(points_near.size());
                for(auto it = points_near.begin();it != points_near.end(); it++){
                    nn.push_back(ToVec3d(*it));
                }
                Vec4d n;
                if (!wxpiggy::math::FitPlane(nn, n)) {
                    // 失败的不要
                    effect_pts[idx] = false;
                    return;
                }
                double dis = n.head<3>().dot(qs) + n[3];
                // double dis_sq = dis * dis;  // 距离的平方

                // 使用类似您参考代码的判断条件
                // bool valid_corr = qs.norm() > 81 * dis_sq;  // 81 * 距离平方
                if (std::fabs(dis) > 0.05) {
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

        Vec6d dx = H.ldlt().solve(err);
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
   
}
}  // namespace wxpiggy