
cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_lib src/husky_trajectory.cpp)

cs_add_executable(${PROJECT_NAME}_node src/husky_trajectory_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib) 

cs_install()
cs_export()