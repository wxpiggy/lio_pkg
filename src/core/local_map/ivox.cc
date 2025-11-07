#include "ivox.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>

namespace wxpiggy {

template class IVox<3, IVoxNodeType::DEFAULT, PointType>;

// IVoxNode implementation
template <typename PointT, int dim>
void IVoxNode<PointT, dim>::InsertPoint(const PointT& pt) {
    points_.emplace_back(pt);
}

template <typename PointT, int dim>
bool IVoxNode<PointT, dim>::Empty() const {
    return points_.empty();
}

template <typename PointT, int dim>
std::size_t IVoxNode<PointT, dim>::Size() const {
    return points_.size();
}

template <typename PointT, int dim>
PointT IVoxNode<PointT, dim>::GetPoint(const std::size_t idx) const {
    return points_[idx];
}

template <typename PointT, int dim>
bool IVoxNode<PointT, dim>::NNPoint(const PointT& point, DistPoint& dist_point) {
    if (points_.empty()) {
        return false;
    }

    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (size_t i = 0; i < points_.size(); ++i) {
        double d = distance2(points_[i], point);
        if (d < min_dist) {
            min_dist = d;
            min_idx = i;
        }
    }

    if (min_idx >= 0) {
        dist_point = DistPoint(min_dist, this, min_idx);
        return true;
    }
    return false;
}

template <typename PointT, int dim>
int IVoxNode<PointT, dim>::KNNPointByCondition(std::vector<DistPoint>& dis_points, const PointT& point, const int& K,
                                               const double& max_range) {
    std::size_t old_size = dis_points.size();

    for (const auto& pt : points_) {
        double d = distance2(pt, point);
        if (d < max_range * max_range) {
            dis_points.emplace_back(DistPoint(d, this, &pt - points_.data()));
        }
    }

    // sort by distance
    if (old_size + K >= dis_points.size()) {
        // Keep all candidates
    } else {
        std::nth_element(dis_points.begin() + old_size, dis_points.begin() + old_size + K - 1, dis_points.end());
        dis_points.resize(old_size + K);
    }

    return dis_points.size();
}

// IVox implementation
template <int dim, IVoxNodeType node_type, typename PointType>
IVox<dim, node_type, PointType>::IVox(Options options) : options_(options) {
    options_.inv_resolution_ = 1.0 / options_.resolution_;
    GenerateNearbyGrids();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    std::for_each(std::execution::unseq, points_to_add.begin(), points_to_add.end(), [this](const auto& pt) {
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            grids_map_.insert({key, grids_cache_.begin()});

            grids_cache_.front().second.InsertPoint(pt);

            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }
        } else {
            iter->second->second.InsertPoint(pt);
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();
        }
    });
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    
    for (const auto& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    }

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(ToEigen<float, dim>(pt));

    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            auto tmp = iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    if (candidates.size() <= max_num) {
        // Keep all candidates
    } else {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    closest_pt.clear();
    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return !closest_pt.empty();
}


template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    nearby_grids_.clear();
    
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        LOG(ERROR) << "Unknown nearby_type!";
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}



// Explicit instantiations
template class IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZ>;
template class IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>;
template class IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZINormal>;

}  // namespace faster_lio