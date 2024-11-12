cmake_minimum_required(VERSION 3.0.2)
project(aonehorn_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  common_lib
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

# OpenCV 패키지 찾기 추가
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aonehorn_detection
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  # OpenCV 경로 추가
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/cone_detection.cpp
  src/segmap.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}  # OpenCV 라이브러리 링크
)
