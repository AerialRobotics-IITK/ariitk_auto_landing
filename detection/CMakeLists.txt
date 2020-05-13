cmake_minimum_required(VERSION 2.8.3)
project(detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_detect_platform_lib src/detect_platform.cpp)

cs_add_executable(${PROJECT_NAME}_detect_platform_node src/detect_platform_node.cpp)

target_link_libraries(${PROJECT_NAME}_detect_platform_node ${PROJECT_NAME}_detect_platform_lib)

cs_install()
cs_export()
