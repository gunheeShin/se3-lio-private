cmake_minimum_required(VERSION 3.16...3.26)
project(se3_lio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/se3_lio ${CMAKE_CURRENT_BINARY_DIR}/se3_lio)

option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" ON)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/se3_lio/thirdparty/find_dependencies.cmake)

# ROS 1 (catkin build or catkin_make)
if (DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "ROS1 build")
    # Select the ROS1 pipeline at build time, no source edits:
    #   catkin build se3_lio --cmake-args -DROS1_PIPELINE=ros1
    # ros1      : online streaming (live topics)
    # ros1_base : offline file-based (cvgl pipeline)
    set(ROS1_PIPELINE "ros1_base" CACHE STRING "ROS1 pipeline: ros1 | ros1_base")
    message(STATUS "SE3-LIO ROS1 pipeline: ${ROS1_PIPELINE}")
    add_subdirectory(${ROS1_PIPELINE})

# ROS 2 (colcon build)
elseif (DEFINED ENV{AMENT_PREFIX_PATH})
    message(STATUS "ROS2 build")
    add_subdirectory(ros2)

# C++ (cmake .. && make)
else()
    message(STATUS "C++ build")

endif()
