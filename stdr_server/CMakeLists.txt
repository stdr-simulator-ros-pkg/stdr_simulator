cmake_minimum_required(VERSION 2.8.3)
project(stdr_server)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    nav_msgs
    stdr_msgs
    actionlib
    nodelet
    map_server
    visualization_msgs
)

set(CMAKE_BUILD_TYPE Release)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    stdr_map_loader
  CATKIN_DEPENDS
    roscpp
    tf
    nav_msgs
    stdr_msgs
    actionlib
    nodelet
    map_server
    visualization_msgs
)

link_directories(${catkin_LIBRARY_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS})

add_library(stdr_map_loader src/map_loader.cpp)
target_link_libraries(stdr_map_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

add_library(stdr_map_server src/map_server.cpp)
target_link_libraries(stdr_map_server
    stdr_map_loader
    ${catkin_LIBRARIES}
)

add_library(stdr_server src/stdr_server.cpp)
add_dependencies(stdr_server ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(stdr_server
  stdr_map_server
  ${catkin_LIBRARIES}
)

add_executable(stdr_server_node src/stdr_server_node.cpp)
target_link_libraries(stdr_server_node
  stdr_server
  ${catkin_LIBRARIES}
)

add_executable(load_map src/map_loader_node.cpp)
add_dependencies(load_map ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(load_map
  stdr_map_loader
  ${catkin_LIBRARIES}
)

# Testing
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Install libraries
install(TARGETS
    stdr_map_loader
    stdr_map_server
    stdr_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install excecutables
install(TARGETS
    stdr_server_node
    load_map
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES include/${PROJECT_NAME}/map_loader.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
