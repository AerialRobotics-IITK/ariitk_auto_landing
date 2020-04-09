cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_lib src/tracking.cpp)


#cs_add_library(auto_landing_lib src/template_ros.cpp src/template_ros_pub.cpp)
#target_link_libraries(auto_landing_lib ${PCL_LIBRARIES})

cs_add_library(${PROJECT_NAME}_lib1 src/landing.cpp)


#cs_add_executable(auto_landing_node src/template_ros_node.cpp)
#target_link_libraries(auto_landing_node auto_landing_lib) 

cs_add_executable(${PROJECT_NAME}_node src/tracking_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib) 

cs_add_executable(landing_node src/landing_node.cpp)
target_link_libraries(landing_node ${PROJECT_NAME}_lib1)
 
cs_install()
cs_export()
