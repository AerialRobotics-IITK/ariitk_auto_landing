cmake_minimum_required(VERSION 2.8.3)
project(user_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
#

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES user_interface
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(Curses REQUIRED)

add_executable(user_interface 
  src/user_interface.cpp
)

target_link_libraries(user_interface 
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)