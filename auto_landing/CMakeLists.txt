cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_trajectory_lib src/trajectory_publisher_visualizer.cpp)

cs_add_executable(${PROJECT_NAME}_trajectory_publisher_visualizer src/trajectory_publisher_visualizer_node.cpp)

target_link_libraries(${PROJECT_NAME}_trajectory_publisher_visualizer ${PROJECT_NAME}_trajectory_lib)

cs_install()
cs_export()