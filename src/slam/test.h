#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

struct VGICPParams {
    public:
    bool visualize = false;
    int num_threads = 4;
    int num_neighbors = 20;
    double downsampling_resolution = 0.25;
    double voxel_resolution = 1.0;
    double max_correspondence_distance = 1.0;
};
class incremental_vgicp{
    VGICPParams params;
    std::shared_ptr<small_gicp::GaussianVoxelMap> voxelmap = std::make_shared<small_gicp::GaussianVoxelMap>(params.voxel_resolution);
};