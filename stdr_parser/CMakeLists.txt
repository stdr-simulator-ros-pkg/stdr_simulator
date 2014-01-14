cmake_minimum_required(VERSION 2.8.3)
project(stdr_parser)

set(CMAKE_BUILD_TYPE Debug)
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
add_executable(stdr_parser_node 
  src/stdr_parser_ros_node.cpp
  src/stdr_parser.cpp
  src/stdr_parser_validator.cpp
  src/stdr_parser_msg_creator.cpp
  src/stdr_parser_file_writer.cpp
  src/stdr_parser_node.cpp
  src/stdr_parser_specs.cpp
  src/stdr_parser_tools.cpp
)

add_dependencies(stdr_parser_node stdr_msgs_gencpp) # wait for stdr_msgs to be build

target_link_libraries(stdr_parser_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
