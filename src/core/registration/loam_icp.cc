#include "core/registration/loam_icp.h"
namespace wxpiggy {
void LoamICP::AddCloud(const std::initializer_list<CloudPtr>& cloud_world){// param not used , only here because registration_base

}


void LoamICP::SetSource(const std::initializer_list<CloudPtr>& source){
    source_surf_ = *source.begin();
    source_line_ = *(source.begin() + 1);
}


bool LoamICP::Align(SE3& init_pose){

}

void LoamICP::Init(){
    
}
}
