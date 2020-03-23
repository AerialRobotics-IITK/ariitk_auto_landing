cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)


cs_add_library(auto_landing_lib src/tracking.cpp)

cs_add_executable(auto_landing_node src/tracking_node.cpp)
target_link_libraries(auto_landing_node auto_landing_lib) 

cs_install()
cs_export()