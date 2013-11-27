cmake_minimum_required(VERSION 2.8.3)
project(stdr_gui)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	tf
	stdr_msgs
	stdr_robot
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
		QtCore 
		QtGui 
		QtUiTools
    CATKIN_DEPENDS # TODO
		roscpp
		roslib
		tf
		stdr_msgs
		stdr_robot
		sensor_msgs
		nav_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
    
)

#----------------------------------------------------------------------------------------------------#
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_gui_controller.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_gui_connector.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_info_connector.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_map_connector.h)
QT_WRAP_CPP(stdr_gui_node SAMPLE_MOC_SRCS include/stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.h)

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/stdr_gui/*.cpp)
#----------------------------------------------------------------------------------------------------#

set(CMAKE_BUILD_TYPE Debug)

add_executable(stdr_gui_node
	src/stdr_gui/stdr_gui_node.cpp
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
	src/stdr_gui/stdr_robot_creator/stdr_rfid_antenna_properties_loader.cpp
	src/stdr_gui/stdr_robot_creator/stdr_robot_creator_loader.cpp
	src/stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.cpp
	src/stdr_gui/stdr_gui_sensors/stdr_gui_robot.cpp
	src/stdr_gui/stdr_gui_sensors/stdr_gui_laser.cpp
	src/stdr_gui/stdr_gui_sensors/stdr_gui_sonar.cpp
	src/stdr_gui/stdr_gui_sensors/stdr_gui_rfid.cpp
	${QT_SOURCES} ${QT_FORMS_HPP} ${SAMPLE_MOC_SRCS}
)
add_dependencies(stdr_gui_node stdr_msgs_gencpp stdr_robot_gencpp)	# Wait for std messages to build
target_link_libraries(stdr_gui_node 
	${catkin_LIBRARIES} 
	${QT_LIBRARIES}
)
