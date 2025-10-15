//
// Created by BowenBZ on 2023/5/10.
//
#define PCL_NO_PRECOMPILE
#include "point_cloud_utils.h"


#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

/// 点云的一些工具函数
template class pcl::VoxelGrid<wxpiggy::PointType>;
namespace wxpiggy {

/// 写点云文件
template<typename CloudType> 
void SaveCloudToFile(const std::string &filePath, CloudType &cloud) {
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileASCII(filePath, cloud);
}

template void SaveCloudToFile<PointCloudType>(const std::string &filePath, PointCloudType &cloud);

template void SaveCloudToFile<FullPointCloudType>(const std::string &filePath, FullPointCloudType &cloud);

}  // namespace sad