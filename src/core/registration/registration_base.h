
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
        GICP,   
    };
    virtual ~RegistrationBase() = default;


    virtual void AddCloud(CloudPtr cloud_world) = 0;


    virtual void SetSource(CloudPtr source) = 0;


    virtual bool Align(SE3& init_pose) = 0;


    virtual void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) = 0;
    virtual void Init() = 0;
};

}  // namespace wxpiggy