cmake_minimum_required(VERSION 2.8.3)
project(stdr_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  actionlib
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES stdr_robot
  CATKIN_DEPENDS
    roscpp
	nodelet
	actionlib
	tf
	stdr_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
#  DEPENDS system_lib
)

######################### Sensors ######################################
add_library(laser src/sensors/laser.cpp)
add_dependencies(laser stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(laser ${catkin_LIBRARIES})

###################### Motion Controller ###############################
add_library(ideal_motion_controller src/motion/ideal_motion_controller.cpp)
add_dependencies(ideal_motion_controller stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(ideal_motion_controller ${catkin_LIBRARIES})

######################### Robot ########################################
add_library(stdr_robot_nodelet src/stdr_robot.cpp)
add_dependencies(stdr_robot_nodelet stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_robot_nodelet ${catkin_LIBRARIES}
	laser
	ideal_motion_controller
)

## Mark the nodelet library for installations
install(TARGETS stdr_robot_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
