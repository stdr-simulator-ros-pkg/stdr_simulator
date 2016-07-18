/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

   Authors :
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com
******************************************************************************/

#include "stdr_server/map_loader.h"
#include <stdr_msgs/LoadExternalMap.h>

#define USAGE "USAGE: load_map <map_file.yaml>"

/**
@brief Main function of the server node
@param argc [int] Number of input arguments
@param argv [char**] Input arguments
@return int
**/
int main(int argc, char** argv) {

  ros::init(argc, argv, "map_loader", ros::init_options::AnonymousName);

  ros::NodeHandle nh;

  if (argc == 2) {

    nav_msgs::OccupancyGrid map;

    map = stdr_server::map_loader::loadMap(std::string(argv[1]));

    ros::ServiceClient client;

    while (!ros::service::waitForService(
      "/stdr_server/load_static_map_external", ros::Duration(.1)) && ros::ok())
    {
      ROS_WARN(
        "Trying to register to /stdr_server/load_static_map_external...");
    }
    client = nh.serviceClient<stdr_msgs::LoadExternalMap>
      ("/stdr_server/load_static_map_external", true);

    stdr_msgs::LoadExternalMap srv;

    srv.request.map = map;

    if (client.call(srv)) {
      ROS_INFO("Map successfully loaded");
      return 0;
    }
    else {
      ROS_ERROR("Could not load map, maybe already loaded...");
      return -1;
    }

  }
  else {
    ROS_ERROR("%s", USAGE);
    return -1;
  }

}
