cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_pose_lib src/pose_estimation_ros.cpp)

cs_add_executable(${PROJECT_NAME}_pose_node src/pose_estimation_ros_node.cpp)

target_link_libraries(${PROJECT_NAME}_pose_node ${PROJECT_NAME}_pose_lib)

cs_install()
cs_export()
