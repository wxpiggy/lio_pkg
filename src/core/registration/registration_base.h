
#pragma once

#include "common/eigen_types.h"
#include "common/point_types.h"

namespace wxpiggy {

/// 注册器基类接口
class RegistrationBase {
public:
    enum class RegistraionType {
        LOAM = 1, 
        NDT,    
        POINT2PLANE,   
    };
    virtual ~RegistrationBase() = default;


    virtual void AddCloud(const std::initializer_list<CloudPtr>& cloud_world) = 0;


    virtual void SetSource(const std::initializer_list<CloudPtr>& source) = 0;


    virtual bool Align(SE3& init_pose) = 0;


    virtual void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr, bool nearest_search ) = 0;
    virtual void Init() = 0;
};

}  // namespace wxpiggy