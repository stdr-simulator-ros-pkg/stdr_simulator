cmake_minimum_required(VERSION 2.8.3)
project(stdr_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
					roscpp
					tf
					nav_msgs
					stdr_msgs
					map_server
)


find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
    CATKIN_DEPENDS
        roscpp
        tf
        nav_msgs
)

link_directories(${catkin_LIBRARY_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS})


add_library(map_server src/stdr_server/map_server.cpp)
target_link_libraries(map_server
    image_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

add_library(stdr_server src/stdr_server/stdr_server.cpp)
add_dependencies(stdr_server stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(stdr_server
	map_server
	${catkin_LIBRARIES}
)

add_executable(stdr_server_node src/stdr_server/stdr_server_node.cpp)
target_link_libraries(stdr_server_node
	stdr_server
	${catkin_LIBRARIES}
)

