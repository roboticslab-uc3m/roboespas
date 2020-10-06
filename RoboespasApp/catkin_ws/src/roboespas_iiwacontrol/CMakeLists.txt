cmake_minimum_required(VERSION 2.8.3)
project(roboespas_iiwacontrol)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  iiwa_msgs
  iiwa_ros
  trajectory_msgs
  message_runtime
  actionlib_msgs
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED NO_MODULE)


## Uncomment this if the package has a setup.py.
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
	FILES
    #Movement
	SendJointPositionTrajectory.srv
	SendJointVelocityTrajectory.srv
	SendJointPosition.srv
	SendIDKVelocityTrajectory.srv
	#Simulation
	SendJointPositionGazebo.srv
	SendJointTrajectoryGazebo.srv
	SendCurrentIiwaPositionToGazebo.srv
	#Configuration
	SetImpedanceControlMode.srv
	SetPositionControlMode.srv
	#Capture
	StartCapture.srv
	StopCapture.srv
	SaveCapture.srv
	ClearCapture.srv
	LoadCapture.srv
    NewCapture.srv
    ListCapture.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   trajectory_msgs
   geometry_msgs
   std_msgs
   iiwa_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roboespas
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
#   DEPENDS eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(${Eigen_INCLUDE_DIRS})
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include)

## Declare a C++ library
add_library(screw_theory_lib src/screw_theory_lib.cpp)
add_library(capture_lib src/capture_lib.cpp)
add_library(filter_lib src/filter_lib.cpp)

## Add cmake target dependencies of the library


## Declare a C++ executable

add_executable(movement src/movement.cpp)
add_executable(configuration src/configuration.cpp)
add_executable(capture src/capture.cpp)
add_executable(simulation src/simulation.cpp)
## Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(movement ${catkin_LIBRARIES} capture_lib)
target_link_libraries(configuration ${catkin_LIBRARIES})
target_link_libraries(capture ${catkin_LIBRARIES} capture_lib)
target_link_libraries(simulation ${catkin_LIBRARIES} capture_lib)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roboespas.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
