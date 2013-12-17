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
  INCLUDE_DIRS 
    include
  LIBRARIES 
	handle_robot
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
add_library(sonar src/sensors/sonar.cpp)
add_dependencies(sonar stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(sonar ${catkin_LIBRARIES})

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
        sonar
	ideal_motion_controller
)

######################### HandleRobot ##################################
add_library(handle_robot src/handle_robot.cpp)
add_dependencies(handle_robot stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(handle_robot ${catkin_LIBRARIES})

####################### Robot Handler ##################################
add_executable(robot_handler src/handle_robot_node.cpp)
add_dependencies(robot_handler stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(robot_handler 
	${catkin_LIBRARIES}
	handle_robot
)

## Mark the nodelet library for installations
install(TARGETS stdr_robot_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
