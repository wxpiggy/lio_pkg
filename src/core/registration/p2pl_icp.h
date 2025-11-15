#pragma once
#include <tsl/robin_map.h>

#include <list>

#include "common/eigen_types.h"
#include "common/point_types.h"
#include "core/registration/registration_base.h"
#include "core/local_map/ivox/ivox3d.h"
namespace wxpiggy {
class IncIcp3d  :public RegistrationBase {
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
   public:
    enum class NearbyType {
        CENTER = 0 ,    // 只考虑中心
        NEARBY6 = 6 ,   
        NEARBY18 = 18,   
        NEARBY26 = 26,  
    };

    struct Options {
        int max_iteration_ = 15;        // ICP 最大迭代次数
        double voxel_size_ = 0.5;      // 体素大小
        double inv_voxel_size_ = 2.0;  // 体素大小之逆
        int min_effective_pts_ = 10;   
        double eps_ = 0.01;         // 收敛判定条件
        size_t capacity_ = 500000;  // LRU 最大容量
        size_t max_points_ = 20;

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };
    /// 获取统计信息
    inline int NumGrids() const { 
         return ivox_->NumValidGrids();
     }

    /// 添加点云到local map
    void AddCloud(const std::initializer_list<CloudPtr>& cloud_world) override;

    /// 设置源点云
    void SetSource(const std::initializer_list<CloudPtr>& source) override{ 
        source_ = *source.begin();
     }

    /// 点到面 ICP 配准
    bool Align(SE3& init_pose)override;
    /**
     * 计算残差和雅可比
     * @param pose 当前位姿
     * @param HTVH 累积的 Hessian
     * @param HTVr 累积的梯度
     */
    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr, bool nearest_search) override ;

    void Init() override;    
    double computeConditionNumber(const Eigen::MatrixXd& H);
   private:
    void GenerateNearbyGrids();

    CloudPtr source_ = nullptr;
    Options options_;
    
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;
    bool flag_first_scan_ = false;
    std::vector<std::vector<Point, Eigen::aligned_allocator<Point>>> nearest_points_;
    float filter_size_map_min_ =0.5;
    bool is_converged_ = true;;
};
}  // namespace wxpiggy