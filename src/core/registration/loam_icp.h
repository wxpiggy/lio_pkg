#pragma once

#include "common/eigen_types.h"
#include "common/point_types.h"

#include "core/registration/registration_base.h"


#include <list>
namespace wxpiggy {
    class LoamICP : public RegistrationBase{
        public:
        void AddCloud(CloudPtr cloud_world);


        void SetSource(CloudPtr source);


        bool Align(SE3& init_pose);


        void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr){

        };
        void Init();
        private:
        CloudPtr source_;
    };
}