cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_gensrv()

rosbuild_add_executable(ocv_capture src/ocv_capture.cpp)
rosbuild_add_executable(ocv_canny src/ocv_canny.cpp)
