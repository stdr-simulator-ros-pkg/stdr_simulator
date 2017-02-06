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
)

set(CMAKE_BUILD_TYPE Release)

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
#  DEPENDS system_lib
)

######################### Sensors ######################################
add_library(stdr_sensor_base src/sensors/sensor_base.cpp)
target_link_libraries(stdr_sensor_base ${catkin_LIBRARIES})

add_library(stdr_sonar src/sensors/sonar.cpp)
add_dependencies(stdr_sonar ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_sonar ${catkin_LIBRARIES} stdr_sensor_base)

add_library(stdr_rfid_reader src/sensors/rfid_reader.cpp)
add_dependencies(stdr_rfid_reader ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_rfid_reader ${catkin_LIBRARIES} stdr_sensor_base)

add_library(stdr_co2_sensor src/sensors/co2.cpp)
add_dependencies(stdr_co2_sensor ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_co2_sensor ${catkin_LIBRARIES} stdr_sensor_base)

add_library(stdr_microphone_sensor src/sensors/microphone.cpp)
add_dependencies(stdr_microphone_sensor ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_microphone_sensor ${catkin_LIBRARIES} stdr_sensor_base)

add_library(stdr_thermal_sensor src/sensors/thermal.cpp)
add_dependencies(stdr_thermal_sensor ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_thermal_sensor ${catkin_LIBRARIES} stdr_sensor_base)

add_library(stdr_laser src/sensors/laser.cpp)
add_dependencies(stdr_laser ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_laser ${catkin_LIBRARIES} stdr_sensor_base)

###################### Motion Controller ###############################
add_library(stdr_ideal_motion_controller src/motion/ideal_motion_controller.cpp)
add_dependencies(stdr_ideal_motion_controller ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_ideal_motion_controller ${catkin_LIBRARIES})

add_library(stdr_omni_motion_controller src/motion/omni_motion_controller.cpp)
add_dependencies(stdr_omni_motion_controller ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_omni_motion_controller ${catkin_LIBRARIES})

######################### Robot ########################################
add_library(stdr_robot_nodelet src/stdr_robot.cpp)
add_dependencies(stdr_robot_nodelet ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_robot_nodelet ${catkin_LIBRARIES}
    stdr_laser
    stdr_sonar
    stdr_rfid_reader
    stdr_co2_sensor
    stdr_thermal_sensor
    stdr_microphone_sensor
    stdr_ideal_motion_controller
    stdr_omni_motion_controller
)

######################### HandleRobot ##################################
add_library(stdr_handle_robot src/handle_robot.cpp)
add_dependencies(stdr_handle_robot ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_handle_robot ${catkin_LIBRARIES})

####################### Robot Handler ##################################
add_executable(robot_handler src/handle_robot_node.cpp)
target_link_libraries(robot_handler
  ${catkin_LIBRARIES}
  stdr_handle_robot
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
    stdr_sensor_base
    stdr_sonar
    stdr_rfid_reader
    stdr_co2_sensor
    stdr_microphone_sensor
    stdr_thermal_sensor
    stdr_laser
    stdr_ideal_motion_controller
    stdr_omni_motion_controller
    stdr_handle_robot
    stdr_robot_nodelet
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install excecutables
install(TARGETS robot_handler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
