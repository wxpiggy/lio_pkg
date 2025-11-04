//
// Created by xiang on 2022/3/15.
//

#pragma once

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/point_types.h"

namespace wxpiggy {

inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::PointCloud2::Ptr msg) {
    CloudPtr cloud(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

/**
 * 其他类型点云转到PointType点云
 * 用的最多的是全量点云转到XYZI点云
 * @tparam PointT
 * @param input
 * @return
 */
template <typename PointT = FullPointType>
CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
    CloudPtr cloud(new PointCloudType);
    for (auto& pt : input->points) {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        p.ring = pt.ring;
        cloud->points.emplace_back(p);
    }
    cloud->width = input->width;
    return cloud;
}

/// 对点云进行voxel filter,指定分辨率
inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
}

template <typename S, int n>
inline Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1>& value) {
    return value.array().round().template cast<int>();
}

}  // namespace wxpiggy


