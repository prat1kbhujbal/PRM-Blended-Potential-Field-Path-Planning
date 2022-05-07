cmake_minimum_required(VERSION 3.0.2)
project(prm_apf_planner)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  roscpp
  rospy
  xacro
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS xacro gazebo_ros
)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(./include)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY 
  launch 
  worlds 
  models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE)
