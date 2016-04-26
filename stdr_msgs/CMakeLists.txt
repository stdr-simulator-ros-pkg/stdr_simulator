cmake_minimum_required(VERSION 2.8.3)
project(stdr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  nav_msgs
)

add_message_files(
    FILES
    Noise.msg
    LaserSensorMsg.msg
    SonarSensorMsg.msg
    KinematicMsg.msg
    FootprintMsg.msg
    RobotMsg.msg
    RobotIndexedMsg.msg
    RobotIndexedVectorMsg.msg

    RfidSensorMsg.msg
    RfidSensorMeasurementMsg.msg
    RfidTag.msg
    RfidTagVector.msg

    SoundSensorMsg.msg
    SoundSensorMeasurementMsg.msg
    SoundSource.msg
    SoundSourceVector.msg

    ThermalSensorMsg.msg
    ThermalSensorMeasurementMsg.msg
    ThermalSource.msg
    ThermalSourceVector.msg

    CO2SensorMsg.msg
    CO2SensorMeasurementMsg.msg
    CO2Source.msg
    CO2SourceVector.msg
)

add_service_files(
    FILES
    LoadMap.srv
    LoadExternalMap.srv
    RegisterGui.srv
    MoveRobot.srv

    AddRfidTag.srv
    DeleteRfidTag.srv

    AddThermalSource.srv
    DeleteThermalSource.srv

    AddSoundSource.srv
    DeleteSoundSource.srv

    AddCO2Source.srv
    DeleteCO2Source.srv
)

add_action_files(
    FILES
    RegisterRobot.action
    SpawnRobot.action
    DeleteRobot.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stdr_msgs
 CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    actionlib_msgs
    nav_msgs
#  DEPENDS system_lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


