cmake_minimum_required(VERSION 2.8.3)
project(iiwa_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(gazebo REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(iiwa_gazebo_plugin
  src/iiwa_gazebo_plugin.cpp
)
