cmake_minimum_required(VERSION 2.8.3)
project(stdr_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation 
  std_msgs
  geometry_msgs
  actionlib_msgs
)



add_message_files(
	FILES
	Noise.msg
	LaserSensorMsg.msg
	SonarSensorMsg.msg
	RfidSensorMsg.msg
	KinematicMsg.msg
	RobotMsg.msg
	RobotIndexedMsg.msg
)

add_service_files( 
	FILES
	LoadMap.srv
)

add_action_files(
	FILES
	RegisterRobot.action
	SpawnRobot.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
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
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


