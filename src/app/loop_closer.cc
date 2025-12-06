#include "slam/backend/loop_closure.h"
#include "tools/config.h"
int main(int argc, char** argv) {
    std::shared_ptr<wxpiggy::LoopClosure> lc_ptr = std::make_shared< wxpiggy::LoopClosure>();

    Config::GetInstance().LoadConfig("/catkin_ws/src/lio_pkg/config/velodyne_nclt.yaml");
    lc_ptr->Init();
    lc_ptr->Run();
    // Config::LoopConfig loop_cofig;
    return 0;

}