#pragma once

#include "common/eigen_types.h"
#include "common/point_types.h"

#include "core/registration/registration_base.h"


#include <list>
namespace wxpiggy {
    class LoamICP : public RegistrationBase{
        public:
        void AddCloud(const std::initializer_list<CloudPtr>& cloud_world) override;


        void SetSource(const std::initializer_list<CloudPtr>& source) override;


        bool Align(SE3& init_pose) override;


        void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr, bool nearest_search) override{

        };
        void Init();
        private:
        CloudPtr source_surf_;
        CloudPtr source_line_;
    };
}