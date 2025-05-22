cmake_minimum_required(VERSION 2.8.3)
project(jacobian)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reemc_visualizations
#  CATKIN_DEPENDS kdl_parser roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/reemc_visualizations_node.cpp
# )
#on my docker container I needed to do this to link mismatched orocos kdl libs...
#sudo ln -s /opt/ros/kinetic/lib/liborocos-kdl.so /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.2

catkin_install_python(PROGRAMS scripts/jacobian.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
