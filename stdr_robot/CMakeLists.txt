cmake_minimum_required(VERSION 2.8.3)
project(stdr_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  actionlib
  tf
  stdr_msgs
  stdr_parser
  geometry_msgs
  sensor_msgs
  nav_msgs
  stdr_parser
)

set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    stdr_handle_robot
  CATKIN_DEPENDS
    roscpp
    nodelet
    actionlib
    tf
    stdr_msgs
    stdr_parser
    geometry_msgs
    sensor_msgs
    nav_msgs
    stdr_parser
#  DEPENDS system_lib
)

######################### Sensors ######################################
add_library(stdr_sonar src/sensors/sonar.cpp)
add_dependencies(stdr_sonar stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_sonar ${catkin_LIBRARIES})

add_library(stdr_laser src/sensors/laser.cpp)
add_dependencies(stdr_laser stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_laser ${catkin_LIBRARIES})

###################### Motion Controller ###############################
add_library(stdr_ideal_motion_controller src/motion/ideal_motion_controller.cpp)
add_dependencies(stdr_ideal_motion_controller stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_ideal_motion_controller ${catkin_LIBRARIES})

######################### Robot ########################################
add_library(stdr_robot_nodelet src/stdr_robot.cpp)
add_dependencies(stdr_robot_nodelet stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_robot_nodelet ${catkin_LIBRARIES}
    stdr_laser
    stdr_sonar
    stdr_ideal_motion_controller
)

######################### HandleRobot ##################################
add_library(stdr_handle_robot src/handle_robot.cpp)
add_dependencies(stdr_handle_robot stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_handle_robot ${catkin_LIBRARIES})

####################### Robot Handler ##################################
add_executable(robot_handler src/handle_robot_node.cpp)
add_dependencies(robot_handler stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(robot_handler 
	${catkin_LIBRARIES}
	stdr_handle_robot
  stdr_parser
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES robot_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Insall libraries
install(TARGETS 
    stdr_sonar
    stdr_laser
    stdr_ideal_motion_controller
    stdr_handle_robot
    stdr_robot_nodelet
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY 
    include/${PROJECT_NAME}/motion
    include/${PROJECT_NAME}/sensors
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install excecutables
install(TARGETS robot_handler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
