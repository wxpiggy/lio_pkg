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

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../build/devel/include)
# add_subdirectory(src)
add_library(lio_core SHARED
    src/common/io_utils.cc
    src/common/timer/timer.cc
    src/common/global_flags.cc
    src/common/point_cloud_utils.cc
    src/core/static_imu_init.cc
    src/map/ndt_inc.cc
    src/map/icp_inc.cc
    src/preprocess/cloud_convert.cc
    src/preprocess/measure_sync.cc
    src/slam/incremental_ndt_lo.cc
    src/slam/loosely_lio.cc
    src/slam/lio_iekf.cc
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
add_executable(${PROJECT_NAME}_node
    src/app/main_ros.cc
)
add_executable(${PROJECT_NAME}_eskf_ndt
    src/app/main_eskf_ndt.cc
)
add_executable(${PROJECT_NAME}_iekf_node
    src/app/iekf_lio_node.cc
)
target_link_libraries(${PROJECT_NAME}_node lio_core)
target_link_libraries(${PROJECT_NAME}_iekf_node lio_core)
target_link_libraries(${PROJECT_NAME}_eskf_ndt lio_core)
# # 安装规则
# install(TARGETS ${PROJECT_NAME}_node
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
