cmake_minimum_required(VERSION 3.5)
project(lio_pkg)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
# ---------------- 构建类型 ----------------
# 默认使用 Release，如果用户没指定
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 编译优化等级
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")   # Release 推荐 O3, 关闭断言
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")        # Debug 开启调试符号，禁优化
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")  # Release + 调试符号
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")# 追求最小体积
add_definitions(-DMP_PROC_NUM=4 -DMP_EN)
add_definitions("-DCATKIN_ENABLE_TESTING=0")
include(cmake/packages.cmake)
catkin_package()
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../build/devel/include)
# add_subdirectory(src)
add_library(lio_core SHARED
    src/common/io_utils.cc
    src/common/timer/timer.cc
    src/common/global_flags.cc
    src/common/point_cloud_utils.cc
    src/common/g2o_types.cc
    src/core/static_imu_init.cc
    src/core/imu_preintegration.cc
    src/map/ndt_inc.cc
    src/map/icp_inc.cc
    src/map/voxel_map.cc
    src/preprocess/cloud_convert.cc
    src/preprocess/measure_sync.cc
    src/slam/incremental_ndt_lo.cc
    src/slam/loosely_lio.cc
    src/slam/lio_iekf.cc
    src/slam/lio_preinteg.cc
    src/slam/voxel_lio.cc
    src/tools/ui/pangolin_window_impl.cc
    src/tools/ui/pangolin_window.cc
    src/tools/ui/ui_car.cc
    src/tools/ui/ui_cloud.cc
    src/tools/ui/ui_trajectory.cc
)
target_link_libraries(lio_core
   ${third_party_libs}
)


# 创建可执行文件
add_executable( lio_preinteg
    src/app/main_preinteg.cc
)
add_executable(lio_iekf
     src/app/main_iekf.cc
)
add_executable(lio_eskf_ndt_offline
    src/app/main_eskf_ndt_offline.cc
)
add_executable(lio_eskf_ndt_online
    src/app/main_eskf_ndt_online.cc
)


target_link_libraries( lio_preinteg lio_core)
target_link_libraries( lio_iekf lio_core)

target_link_libraries(lio_eskf_ndt_offline lio_core)
target_link_libraries(lio_eskf_ndt_online lio_core)
# # 安装规则
# install(TARGETS ${PROJECT_NAME}_node
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
add_executable(test1 
    src/app/test.cc
)
target_link_libraries(test1 lio_core)