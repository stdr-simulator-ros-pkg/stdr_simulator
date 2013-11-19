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

#ifndef STDR_SERVER_H
#define STDR_SERVER_H

#define USAGE "\nUSAGE: stdr_server <map.yaml>\n" \
              "  map.yaml: map description file\n" 


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <stdr_server/map_server.h>
#include <stdr_msgs/LoadMap.h>
#include <stdr_msgs/SpawnRobotAction.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>

namespace stdr_server {

typedef boost::shared_ptr<MapServer> MapServerPtr;
typedef actionlib::SimpleActionServer<stdr_msgs::SpawnRobotAction> SpawnRobotServer;

class Server {
		
	public:
	
		Server(int argc, char** argv);
		bool loadMapCallback(stdr_msgs::LoadMap::Request& req,
							stdr_msgs::LoadMap::Response& res);
		void spawnRobotCallback();
		
	private:
	
		ros::NodeHandle _nh;
		MapServerPtr _mapServer;
		ros::ServiceServer _loadMapService;
		
		SpawnRobotServer _spawnRobotServer;
};	
	
}


#endif
