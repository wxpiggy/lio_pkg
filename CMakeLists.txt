cmake_minimum_required(VERSION 3.5)
project(lio_pkg)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

add_definitions("-DCATKIN_ENABLE_TESTING=0")
include(cmake/packages.cmake)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/devel/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/build/devel/include)

set(SOURCE_FILE
    src/common/io_utils.cc
    src/common/timer/timer.cc
    src/common/global_flags.cc
    src/common/point_cloud_utils.cc

    src/core/static_imu_init.cc

    src/icp/icp_3d.cc
    src/icp/ndt_3d.cc
    src/icp/ndt_inc.cc
    # src/lidar/feature_extraction.cc
    src/preprocess/cloud_convert.cc
    src/preprocess/measure_sync.cc
    src/map/kdtree.cc


    src/slam/incremental_ndt_lo.cc
    src/slam/loosely_lio.cc

    src/tools/ui/pangolin_window_impl.cc
    src/tools/ui/pangolin_window.cc
    src/tools/ui/ui_car.cc
    src/tools/ui/ui_cloud.cc
    src/tools/ui/ui_trajectory.cc
    # src/slam/loam_like_odom.cc

)

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
    src/app/main_ros.cc
    ${SOURCE_FILE}
)

# 链接所有第三方库（包括 ROS 依赖库和其他第三方库）
target_link_libraries(${PROJECT_NAME}_node
    ${third_party_libs}  # 引用第三方库
)

# 安装规则
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
