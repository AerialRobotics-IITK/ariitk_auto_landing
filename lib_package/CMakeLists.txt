cmake_minimum_required(VERSION 2.8.3)
project(lib_package)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#cs_add_library(ariitk_auto_landing_lib src/template.cpp)
#target_link_libraries(ariitk_auto_landing_lib ${PCL_LIBRARIES})

cs_install()
cs_export()

#if(CATKIN_ENABLE_TESTING)
 #   catkin_add_gtest(ariitk_auto_landing-test test/src/test.cpp)
  #  target_link_libraries(ariitk_auto_landing-test ariik_auto_landing_lib)
#endif()