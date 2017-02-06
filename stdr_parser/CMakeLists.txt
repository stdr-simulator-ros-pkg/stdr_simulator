cmake_minimum_required(VERSION 2.8.3)
project(stdr_parser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  stdr_msgs
  cmake_modules
)

set(CMAKE_BUILD_TYPE Release)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
pkg_check_modules(XML libxml-2.0)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    stdr_parser
  CATKIN_DEPENDS
    roscpp
    roslib
    stdr_msgs
  DEPENDS
    TinyXML
)

######################### Stdr parser ##################################
add_library(stdr_parser 
  src/stdr_parser.cpp
  src/stdr_parser_validator.cpp
  src/stdr_parser_msg_creator.cpp
  src/stdr_parser_xml_file_writer.cpp
  src/stdr_parser_yaml_file_writer.cpp
  src/stdr_xml_parser.cpp
  src/stdr_yaml_parser.cpp
  src/stdr_parser_node.cpp
  src/stdr_parser_specs.cpp
  src/stdr_parser_tools.cpp
)
add_dependencies(stdr_parser ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_parser 
  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
  yaml-cpp
)

install(TARGETS 
  stdr_parser
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
