cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_platform_lib src/platform_trajectory.cpp)
cs_add_library(${PROJECT_NAME}_landing_lib src/landing.cpp)

cs_add_executable(${PROJECT_NAME}_platform_node src/platform_trajectory_node.cpp)
cs_add_executable(${PROJECT_NAME}_landing_node src/landing_node.cpp)

target_link_libraries(${PROJECT_NAME}_platform_node ${PROJECT_NAME}_platform_lib)
target_link_libraries(${PROJECT_NAME}_landing_node ${PROJECT_NAME}_landing_lib)

cs_install()
cs_export()
