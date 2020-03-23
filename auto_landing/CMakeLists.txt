cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)


cs_add_library(auto_landing_lib src/trajectory_test.cpp)

cs_add_executable(trajectory_generation_example src/run_trajectory_test.cpp)
target_link_libraries(trajectory_generation_example auto_landing_lib) 

cs_install()
cs_export()