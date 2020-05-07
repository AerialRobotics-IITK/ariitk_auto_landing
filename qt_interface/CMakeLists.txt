cmake_minimum_required(VERSION 2.8.9)
project(qt_interface)


find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS system-lib
)
#if (UseQt5)
 #       find_package(Qt5 COMPONENTS Core Gui Network Widgets Concurrent REQUIRED)
  #      message("qt5 is used")
   #     # Find Qt5Core and Qt5Widgets
    #    find_package(Qt5Core REQUIRED)
     #   find_package(Qt5Widgets REQUIRED)
      #  # For finding Qt includes
       # include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        #include_directories(${Qt5Widgets_INCLUDE_DIRS})
        #include_directories(${Qt5Core_INCLUDE_DIRS})

#else()
 #       find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork QtWidgets REQUIRED)
  #  include(${QT_USE_FILE})
   #     message("qt4 is used")
#endif()


# qt5_add_resources(QT_RESOURCES_CPP qt_resources/arrows.qrc)
qt5_wrap_cpp(QT_MOC_HPP src/mainwindow.h
						#src/RobotThread.h
						)

include_directories(src ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(qt_ros_ctrl src/mainwindow.cpp
						   src/mainwindow.ui
						   #src/RobotThread.cpp
						   src/main.cpp
						   #${QT_RESOURCES_CPP}
              			   ${QT_MOC_HPP})

target_link_libraries(qt_ros_ctrl ${QT_LIBRARIES}
            		  ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS qt_ros_ctrl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)