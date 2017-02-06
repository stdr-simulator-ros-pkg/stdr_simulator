cmake_minimum_required(VERSION 2.8.3)
project(stdr_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  stdr_msgs
  stdr_server
  stdr_robot
  stdr_parser
  sensor_msgs
  nav_msgs
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
  QtUiTools
)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  DEPENDS
    QT
  CATKIN_DEPENDS
    roscpp
    roslib
    tf
    stdr_msgs
    stdr_server
    stdr_robot
    stdr_parser
    sensor_msgs
    nav_msgs
  #INCLUDE_DIRS
  #LIBRARIES
)

#----------------------------------------------------------------------------------------------------#
set(QT_FORMS
  ui/information.ui             ui/kinematicProperties.ui
  ui/laserProperties.ui         ui/laserVisualization.ui
  ui/map.ui                     ui/rfidAntennaProperties.ui
  ui/robotCreator.ui            ui/robotFootprint.ui
  ui/robotProperties.ui         ui/robotVisualization.ui
  ui/simulator.ui               ui/sonarProperties.ui
  ui/sonarVisualization.ui      ui/co2SensorProperties.ui
  ui/thermalSensorProperties.ui ui/soundSensorProperties.ui
)

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_gui_controller.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_gui_connector.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_info_connector.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_map_connector.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.h)

#----------------------------------------------------------------------------------------------------#

#~ set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

add_executable(stdr_gui_node
  src/stdr_gui/stdr_gui_node.cpp
  src/stdr_gui/stdr_gui_application.cpp
  src/stdr_gui/stdr_gui_connector.cpp
  src/stdr_gui/stdr_gui_loader.cpp
  src/stdr_gui/stdr_gui_controller.cpp
  src/stdr_gui/stdr_info_connector.cpp
  src/stdr_gui/stdr_info_loader.cpp
  src/stdr_gui/stdr_map_connector.cpp
  src/stdr_gui/stdr_map_loader.cpp
  src/stdr_gui/stdr_tools.cpp
  src/stdr_gui/stdr_robot_creator/stdr_kinematic_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_laser_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_sonar_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_robot_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_robot_footprint_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_rfid_antenna_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_co2_sensor_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_thermal_sensor_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_sound_sensor_properties_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_robot_creator_loader.cpp
  src/stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_robot.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_laser.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_sonar.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_rfid.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_co2.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_thermal.cpp
  src/stdr_gui/stdr_gui_sensors/stdr_gui_sound.cpp
  src/stdr_gui/stdr_visualization/stdr_laser_visualization.cpp
  src/stdr_gui/stdr_visualization/stdr_sonar_visualization.cpp
  src/stdr_gui/stdr_visualization/stdr_robot_visualization.cpp
  src/stdr_gui/stdr_map_metainformation/stdr_gui_source.cpp
  src/stdr_gui/stdr_map_metainformation/stdr_gui_co2_source.cpp
  src/stdr_gui/stdr_map_metainformation/stdr_gui_thermal_source.cpp
  src/stdr_gui/stdr_map_metainformation/stdr_gui_rfid_tag.cpp
  src/stdr_gui/stdr_map_metainformation/stdr_gui_sound_source.cpp
  ${QT_FORMS_HPP} ${SAMPLE_MOC_SRCS}
)
add_dependencies(stdr_gui_node ${stdr_msgs_EXPORTED_TARGETS})  # Wait for stdr_msgs to build
target_link_libraries(stdr_gui_node
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

# Install launch files
install(DIRECTORY launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install excecutables
install(TARGETS stdr_gui_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

