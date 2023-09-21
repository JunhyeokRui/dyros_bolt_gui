cmake_minimum_required(VERSION 3.5.0)
project(dyros_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # mujoco_ros_msgs
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
  sensor_msgs
  dyros_bolt_msgs
)

find_package(Qt5Widgets REQUIRED)


catkin_python_setup()

catkin_package(
# INCLUDE_DIRS ${rqt_dyros_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)

set(dyros_gui_SRCS
  src/dyros_gui.cpp
)

set(dyros_gui_HDRS
  include/dyros_gui/dyros_gui.h
)

set(dyros_gui_UIS
  src/dyros_gui.ui
)

qt5_wrap_cpp(dyros_gui_MOCS ${dyros_gui_HDRS})
qt5_wrap_ui(dyros_gui_UIS_H ${dyros_gui_UIS})

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${dyros_gui_SRCS}
  ${dyros_gui_MOCS}
  ${dyros_gui_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

