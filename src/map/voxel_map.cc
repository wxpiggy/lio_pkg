#pragma once
#include "voxel_map.h"

#include <mutex>

#include "common/math_utils.h"
namespace wxpiggy {

void VoxelMapManager::calcBodyCov(Eigen::Vector3d &pb,
                                  const float range_inc,
                                  const float degree_inc,
                                  Eigen::Matrix3d &cov) {
    if (pb[2] == 0) pb[2] = 0.0001;
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}
void VoxelMapManager::BuildHTVH_HTVr(
    const SE3 &pose, 
    const Mat6d cov, 
    Mat18d &HTVH, 
    Vec18d &HTVr) 
{
    // 清理
    cross_mat_list_.clear();
    cross_mat_list_.reserve(feats_down_size_);
    body_cov_list_.clear();
    body_cov_list_.reserve(feats_down_size_);
    pv_list_.clear();
    pv_list_.resize(feats_down_size_);
    // 1. 先计算每个点的协方差和 crossmat
    for (size_t i = 0; i < feats_down_body_->size(); i++) {
        Eigen::Vector3d point_this(
            feats_down_body_->points[i].x, 
            feats_down_body_->points[i].y, 
            feats_down_body_->points[i].z);
        if (point_this[2] == 0) point_this[2] = 0.001;

        Eigen::Matrix3d var;
        calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
        body_cov_list_.push_back(var);

        point_this = extR_ * point_this + extT_;
        Eigen::Matrix3d point_crossmat;
        point_crossmat << math::SKEW_SYM_MATRIX(point_this);
        cross_mat_list_.push_back(point_crossmat);
    }

    // 2. 转到 world，计算每个点的 cov
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(pose.rotationMatrix(), pose.translation(), feats_down_body_, world_lidar);
    for (size_t i = 0; i < feats_down_body_->size(); i++) {
        pointWithVar &pv = pv_list_[i];
        pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
        pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

        Eigen::Matrix3d var = body_cov_list_[i];
        Eigen::Matrix3d point_crossmat = cross_mat_list_[i];
        var = pose.rotationMatrix() * var * pose.rotationMatrix().transpose() +
              (-point_crossmat) * cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() +
              cov.block<3, 3>(3, 3);
        pv.var = var;
        pv.body_var = body_cov_list_[i];
    }

    // 3. 构建残差列表
    ptpl_list_.clear();
    BuildResidualListOMP(pv_list_, ptpl_list_);
    LOG(INFO) << "pv_list_.size() = " << pv_list_.size();
    LOG(INFO) << "ptpl_list_.size() = " << ptpl_list_.size();
    int effct_feat_num = ptpl_list_.size();

    // 4. 初始化 H, R_inv, meas_vec
    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 1.0;

    for (size_t i = 0; i < ptpl_list_.size(); i++) {
        auto &ptpl = ptpl_list_[i];

        Eigen::Vector3d point_this = extR_ * ptpl.point_b_ + extT_;
        Eigen::Matrix3d point_crossmat;
        point_crossmat << math::SKEW_SYM_MATRIX(point_this);

        Eigen::Vector3d point_world = pose * point_this;

        // 3x18 Jacobian
        Eigen::Matrix<double, 3, 18> J;
        J.setZero();
        // 对 p
        J.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        // 对 v
        J.block<3,3>(0,3).setZero();
        // 对 θ
        J.block<3,3>(0,6) = -pose.rotationMatrix() * point_crossmat;
        // 对 b_g, b_a, g 保留0或者根据需要修改
        J.block<3,3>(0,9).setZero();
        J.block<3,3>(0,12).setZero();
        J.block<3,3>(0,15).setZero();

        Eigen::Matrix3d var =
            pose.rotationMatrix() * extR_ * ptpl.body_cov_ * (pose.rotationMatrix() * extR_).transpose();
        double sigma_l = ptpl.normal_.transpose() * var * ptpl.normal_;
        double r_inv = 1.0 / (0.001 + sigma_l);

        // 累加到 18×18
        HTVH += J.transpose() * (ptpl.normal_ * ptpl.normal_.transpose() * r_inv) * J * info_ratio;
        HTVr += -J.transpose() * ptpl.normal_ * ptpl.dis_to_plane_ * r_inv * info_ratio;
    }
}

/**
 * @brief 初始化一个体素地图
 *
 * @param cov 协方差矩阵
 * @param pose 位姿
 */
void VoxelMapManager::BuildVoxelMap(const Mat6d &cov, const SE3 &pose) {
    float voxel_size = config_setting_.max_voxel_size_;
    float planer_threshold = config_setting_.planner_threshold_;
    int max_layer = config_setting_.max_layer_;
    int max_points_num = config_setting_.max_points_num_;
    std::vector<int> layer_init_num = config_setting_.layer_init_num_;

    std::vector<pointWithVar> input_points;

    for (size_t i = 0; i < feats_down_world_->size(); i++) {
        pointWithVar pv;
        pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
        Eigen::Vector3d point_this(
            feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
        Eigen::Matrix3d var;
        calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
        Eigen::Matrix3d point_crossmat;
        point_crossmat << math::SKEW_SYM_MATRIX(point_this);
        //=======================!!!!!!!!!!!!!!!!!!!!!!!! 顺序todo

        var = pose.rotationMatrix() * var * pose.rotationMatrix().transpose() +
              (-point_crossmat) * cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + cov.block<3, 3>(3, 3);
        pv.var = var;
        input_points.push_back(pv);
    }

    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const pointWithVar p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_w[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_++;
        } else {
            VoxelOctoTree *octo_tree =
                new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->quater_length_ = voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_++;
            voxel_map_[position]->layer_init_num_ = layer_init_num;
        }
    }
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        iter->second->init_octo_tree();
    }
}

// Eigen::Vector3d RGBFromVoxel(const V3D &input_point);

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points) {
    float voxel_size = config_setting_.max_voxel_size_;
    float planer_threshold = config_setting_.planner_threshold_;
    int max_layer = config_setting_.max_layer_;
    int max_points_num = config_setting_.max_points_num_;
    std::vector<int> layer_init_num = config_setting_.layer_init_num_;
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const pointWithVar p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_w[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->UpdateOctoTree(p_v);
        } else {
            VoxelOctoTree *octo_tree =
                new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->layer_init_num_ = layer_init_num;
            voxel_map_[position]->quater_length_ = voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            voxel_map_[position]->UpdateOctoTree(p_v);
        }
    }
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list) {
    int max_layer = config_setting_.max_layer_;
    double voxel_size = config_setting_.max_voxel_size_;
    double sigma_num = config_setting_.sigma_num_;
    std::mutex mylock;
    ptpl_list.clear();
    std::vector<PointToPlane> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size());
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
        useful_ptpl[i] = false;
    }

    omp_set_num_threads(4);
#pragma omp parallel for

    for (int i = 0; i < index.size(); i++) {
        pointWithVar &pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_w[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            VoxelOctoTree *current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
            if (!is_sucess) {
                VOXEL_LOCATION near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map_.find(near_position);
                if (iter_near != voxel_map_.end()) {
                    build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {
                mylock.lock();
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
                mylock.unlock();
            } else {
                mylock.lock();
                useful_ptpl[i] = false;
                mylock.unlock();
            }
        }
    }
    for (size_t i = 0; i < useful_ptpl.size(); i++) {

        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        }
    }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv,
                                            const VoxelOctoTree *current_octo,
                                            const int current_layer,
                                            bool &is_sucess,
                                            double &prob,
                                            PointToPlane &single_ptpl) {
    int max_layer = config_setting_.max_layer_;
    double sigma_num = config_setting_.sigma_num_;

    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_w;
    if (current_octo->plane_ptr_->is_plane_) {

        VoxelPlane &plane = *current_octo->plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.center_;
        float dis_to_plane =
            fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
        float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                              (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                              (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius_) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
            J_nq.block<1, 3>(0, 3) = -plane.normal_;
            double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
            sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
            if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                is_sucess = true;
                double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    pv.normal = plane.normal_;
                    single_ptpl.body_cov_ = pv.body_var;
                    single_ptpl.point_b_ = pv.point_b;
                    single_ptpl.point_w_ = pv.point_w;
                    single_ptpl.plane_var_ = plane.plane_var_;
                    single_ptpl.normal_ = plane.normal_;
                    single_ptpl.center_ = plane.center_;
                    single_ptpl.d_ = plane.d_;
                    single_ptpl.layer_ = current_layer;
                    single_ptpl.dis_to_plane_ =
                        plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
                }
                return;
            } else {
                // is_sucess = false;
                return;
            }
        } else {
            // is_sucess = false;
            return;
        }
    } else {

        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {
                    VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
                    build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
                }
            }
            return;
        } else {
            return;
        }
    }
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
// void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec,
//                                       const Eigen::Vector3d &y_vec,
//                                       const Eigen::Vector3d &z_vec,
//                                       geometry_msgs::Quaternion &q) {
// }
void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const CloudPtr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    PointType p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}
void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b) {
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane) {
    plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance_ = Eigen::Matrix3d::Zero();
    plane->center_ = Eigen::Vector3d::Zero();
    plane->normal_ = Eigen::Vector3d::Zero();
    plane->points_size_ = points.size();
    plane->radius_ = 0;
    for (auto pv : points) {
        plane->covariance_ += pv.point_w * pv.point_w.transpose();
        plane->center_ += pv.point_w;
    }
    plane->center_ = plane->center_ / plane->points_size_;
    plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;
    // && evalsReal(evalsMid) > 0.05
    //&& evalsReal(evalsMid) > 0.01
    if (evalsReal(evalsMin) < planer_threshold_) {
        for (int i = 0; i < points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m = (points[i].point_w - plane->center_).transpose() /
                                                      ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
                                                      (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                                                       evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    Eigen::Matrix<double, 1, 3> F_m;
                    F_m << 0, 0, 0;
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs.real() * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_var_ += J * points[i].var * J.transpose();
        }

        plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->min_eigen_value_ = evalsReal(evalsMin);
        plane->mid_eigen_value_ = evalsReal(evalsMid);
        plane->max_eigen_value_ = evalsReal(evalsMax);
        plane->radius_ = sqrt(evalsReal(evalsMax));
        plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) +
                      plane->normal_(2) * plane->center_(2));
        plane->is_plane_ = true;
        plane->is_update_ = true;
        if (!plane->is_init_) {
            plane->id_ = voxel_plane_id;
            voxel_plane_id++;
            plane->is_init_ = true;
        }
    } else {
        plane->is_update_ = true;
        plane->is_plane_ = false;
    }
}

void VoxelOctoTree::init_octo_tree() {
    if (temp_points_.size() > points_size_threshold_) {
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane_ == true) {
            octo_state_ = 0;
            // new added
            if (temp_points_.size() > max_points_num_) {
                update_enable_ = false;
                std::vector<pointWithVar>().swap(temp_points_);
                new_points_ = 0;
            }
        } else {
            octo_state_ = 1;
            cut_octo_tree();
        }
        init_octo_ = true;
        new_points_ = 0;
    }
}

void VoxelOctoTree::cut_octo_tree() {
    if (layer_ >= max_layer_) {
        octo_state_ = 0;
        return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point_w[0] > voxel_center_[0]) {
            xyz[0] = 1;
        }
        if (temp_points_[i].point_w[1] > voxel_center_[1]) {
            xyz[1] = 1;
        }
        if (temp_points_[i].point_w[2] > voxel_center_[2]) {
            xyz[2] = 1;
        }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = new VoxelOctoTree(
                max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_++;
    }
    for (uint i = 0; i < 8; i++) {
        if (leaves_[i] != nullptr) {
            if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_) {
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (leaves_[i]->plane_ptr_->is_plane_) {
                    leaves_[i]->octo_state_ = 0;
                    // new added
                    if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_) {
                        leaves_[i]->update_enable_ = false;
                        std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
                        new_points_ = 0;
                    }
                } else {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree();
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_ = 0;
            }
        }
    }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv) {
    if (!init_octo_) {
        new_points_++;
        temp_points_.push_back(pv);
        if (temp_points_.size() > points_size_threshold_) {
            init_octo_tree();
        }
    } else {
        if (plane_ptr_->is_plane_) {
            if (update_enable_) {
                new_points_++;
                temp_points_.push_back(pv);
                if (new_points_ > update_size_threshold_) {
                    init_plane(temp_points_, plane_ptr_);
                    new_points_ = 0;
                }
                if (temp_points_.size() >= max_points_num_) {
                    update_enable_ = false;
                    std::vector<pointWithVar>().swap(temp_points_);
                    new_points_ = 0;
                }
            }
        } else {
            if (layer_ < max_layer_) {
                int xyz[3] = {0, 0, 0};
                if (pv.point_w[0] > voxel_center_[0]) {
                    xyz[0] = 1;
                }
                if (pv.point_w[1] > voxel_center_[1]) {
                    xyz[1] = 1;
                }
                if (pv.point_w[2] > voxel_center_[2]) {
                    xyz[2] = 1;
                }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->UpdateOctoTree(pv);
                } else {
                    leaves_[leafnum] = new VoxelOctoTree(
                        max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
                    leaves_[leafnum]->layer_init_num_ = layer_init_num_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            } else {
                if (update_enable_) {
                    new_points_++;
                    temp_points_.push_back(pv);
                    if (new_points_ > update_size_threshold_) {
                        init_plane(temp_points_, plane_ptr_);
                        new_points_ = 0;
                    }
                    if (temp_points_.size() > max_points_num_) {
                        update_enable_ = false;
                        std::vector<pointWithVar>().swap(temp_points_);
                        new_points_ = 0;
                    }
                }
            }
        }
    }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw) {
    if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

    int xyz[3] = {0, 0, 0};
    xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    // printf("leafnum: %d. \n", leafnum);

    return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv) {
    if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) ||
        (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_))) {
        new_points_++;
        temp_points_.push_back(pv);
        return this;
    }

    if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_)) {
        int xyz[3] = {0, 0, 0};
        xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
        xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
        xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) {
            return leaves_[leafnum]->Insert(pv);
        } else {
            leaves_[leafnum] = new VoxelOctoTree(
                max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            return leaves_[leafnum]->Insert(pv);
        }
    }
    return nullptr;
}

}  // namespace wxpiggy