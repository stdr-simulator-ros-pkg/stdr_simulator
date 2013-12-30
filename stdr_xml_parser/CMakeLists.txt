cmake_minimum_required(VERSION 2.8.3)
project(stdr_xml_parser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  stdr_msgs
  cmake_modules
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
    roslib
    stdr_msgs
  DEPENDS
    TinyXML
)

######################### Stdr parser ##################################
add_executable(stdr_xml_parser_node 
  src/stdr_xml_parser_ros_node.cpp
  src/stdr_xml_parser_base.cpp
  src/stdr_xml_parser_node.cpp
  src/stdr_xml_parser_specs.cpp
  src/stdr_xml_parser_tools.cpp
)

add_dependencies(stdr_xml_parser_node stdr_msgs_gencpp) # wait for stdr_msgs to be build

target_link_libraries(stdr_xml_parser_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
