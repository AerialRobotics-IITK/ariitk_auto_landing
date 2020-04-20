cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME}_husky_lib src/husky_trajectory.cpp)
cs_add_library(${PROJECT_NAME}_tracking_lib src/tracking.cpp)
cs_add_library(${PROJECT_NAME}_trajectory_generation_tracker_lib src/trajectory_generation_tracker.cpp)
cs_add_library(${PROJECT_NAME}_landing_lib src/landing.cpp)
cs_add_library(${PROJECT_NAME}_pose_lib src/pose.cpp)
cs_add_library(${PROJECT_NAME}_detect_platform_lib src/detect_platform.cpp)

cs_add_executable(${PROJECT_NAME}_husky_node src/husky_trajectory_node.cpp)
cs_add_executable(${PROJECT_NAME}_tracking_node src/tracking_node.cpp)
cs_add_executable(${PROJECT_NAME}_trajectory_generation_tracker src/trajectory_generation_tracker_node.cpp)
cs_add_executable(${PROJECT_NAME}_landing_node src/landing_node.cpp)
cs_add_executable(${PROJECT_NAME}_pose_node src/pose_node.cpp)
cs_add_executable(${PROJECT_NAME}_detect_platform_node src/detect_platform_node.cpp)

target_link_libraries(${PROJECT_NAME}_husky_node ${PROJECT_NAME}_husky_lib)
target_link_libraries(${PROJECT_NAME}_tracking_node ${PROJECT_NAME}_tracking_lib)
target_link_libraries(${PROJECT_NAME}_trajectory_generation_tracker ${PROJECT_NAME}_trajectory_generation_tracker_lib)
target_link_libraries(${PROJECT_NAME}_landing_node ${PROJECT_NAME}_landing_lib)
target_link_libraries(${PROJECT_NAME}_pose_node ${PROJECT_NAME}_pose_lib)
target_link_libraries(${PROJECT_NAME}_detect_platform_node ${PROJECT_NAME}_detect_platform_lib)
target_link_libraries(${PROJECT_NAME}_detect_platform_node ${OpenCV_LIBRARIES})

cs_install()
cs_export()
