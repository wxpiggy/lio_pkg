//
// Created by xiang on 2021/8/25.
//

#pragma once

#include "point_types.h"

/// 点云的一些工具函数

namespace wxpiggy {

/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5);

/// 写点云文件
template<typename CloudType> 
void SaveCloudToFile(const std::string &filePath, CloudType &cloud);

}  // namespace sad


