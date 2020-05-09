cmake_minimum_required(VERSION 2.8.3)
project(gui_rosos)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gui_rosos
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
   src/ROS_GUI.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

