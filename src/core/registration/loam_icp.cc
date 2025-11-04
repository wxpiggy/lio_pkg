#include "core/registration/loam_icp.h"
namespace wxpiggy {
void LoamICP::AddCloud(CloudPtr cloud_world){// param not used , only here because registration_base

}


void LoamICP::SetSource(CloudPtr source){
    source_ = source;
}


bool LoamICP::Align(SE3& init_pose){

}

void LoamICP::Init(){
    
}
}
