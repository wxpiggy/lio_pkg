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
SET(LIB_FILE
    lib/ikdtree/ikdtree.cc
)


add_library(lio_core SHARED
    src/common/timer/timer.cc
    
    src/common/keyframe.cc
    
    src/core/init/static_imu_init.cc
    src/core/optimization/g2o_types.cc
    src/core/optimization/imu_preintegration.cc
    src/core/registration/ndt_inc.cc
    src/preprocess/cloud_convert.cc
    src/preprocess/measure_sync.cc
    src/slam/frontend/incremental_lo.cc
    src/slam/frontend/loosely_lio.cc
    src/slam/frontend/lio_preinteg.cc
    src/tools/point_cloud_utils.cc
    src/tools/utm_convert.cc
    src/tools/config.cc

    thirdparty/utm_convert/utm.cc
    thirdparty/utm_convert/tranmerc.cc
)
target_link_libraries(lio_core
   ${third_party_libs}
   
)

set_target_properties(lio_core PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
)
# 创建可执行文件
add_executable( lio_preinteg
    src/app/main_preinteg.cc
)


add_executable(lio_eskf_online
    src/app/main_eskf_online.cc
)


target_link_libraries( lio_preinteg lio_core)



target_link_libraries(lio_eskf_online lio_core)


# add_executable(test_node 
# src/app/front.cc)

# target_link_libraries(test_node lio_core)
add_executable(map_dumper 
src/app/map_dumper.cc
)
target_link_libraries(map_dumper lio_core)
# # 安装规则
# install(TARGETS ${PROJECT_NAME}_node
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
