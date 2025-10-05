//
// Created by xiang on 2021/8/18.
//

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/impl/pcl_base.hpp>

#include "eigen_types.h"


namespace wxpiggy {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;

    float intensity = 0.0f;
    std::uint32_t offset_time = 0;
    std::int32_t ring = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Point() = default;  // 显式声明默认构造函数，初始化列表用成员默认值完成
};

// 定义系统中用到的点和点云类型
using PointType = Point;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType& pt) {
    return pt.getVector3fMap();
}
inline Vec3d ToVec3d(const PointType& pt) {
    return pt.getVector3fMap().cast<double>();
}

// 模板类型转换函数
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt);

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2>(const PointType& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3>(const PointType& pt) {
    return Vec3f(pt.x, pt.y, pt.z);
}

template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointType p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    return p;
}

/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;
    Eigen::Matrix3d cov;
    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

inline Vec3f ToVec3f(const FullPointType& pt) {
    return pt.getVector3fMap();
}
inline Vec3d ToVec3d(const FullPointType& pt) {
    return pt.getVector3fMap().cast<double>();
}

/// ui中的点云颜色
using UiPointType = pcl::PointXYZRGBA;
using UiPointCloudType = pcl::PointCloud<UiPointType>;
using UiCloudPtr = UiPointCloudType::Ptr;

}  // namespace wxpiggy
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(wxpiggy::FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)(
                                      std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(
                                      std::uint8_t, ring, ring)(double, time, time)(float, height, height))
// clang-format off

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range)
)
// clang-format on

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    wxpiggy::PointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::int32_t, ring, ring)
)
// clang-format off

