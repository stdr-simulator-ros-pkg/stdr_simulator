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

#include <stdr_server/stdr_server.h>

namespace stdr_server {
	
Server::Server(int argc, char** argv) 
  : _spawnRobotServer(_nh, "spawn_robot", false)
  , _registerRobotServer(_nh, "register_robot", false)
  , _deleteRobotServer(_nh, "delete_robot", false)
  , _id(0)
{
	_spawnRobotServer.registerGoalCallback( boost::bind(&Server::spawnRobotCallback, this) );
	
	if (argc > 2) {
		ROS_ERROR("%s", USAGE);
		exit(-1);
	}
	
	if (argc == 2) {
		std::string fname(argv[1]);
		_mapServer.reset(new MapServer(fname));
		// if we don't have map, no point to spawn robot
		_spawnRobotServer.start();
	}
		
	_loadMapService = _nh.advertiseService("/stdr_server/load_static_map", &Server::loadMapCallback, this);
	
	while (!ros::service::waitForService("robot_manager/load_nodelet", ros::Duration(.1)) && ros::ok()) {
		ROS_WARN("Trying to register to robot_manager/load_nodelet...");
	}
	_spawnRobotClient = _nh.serviceClient<nodelet::NodeletLoad>("robot_manager/load_nodelet", true);
	
	while (!ros::service::waitForService("robot_manager/unload_nodelet", ros::Duration(.1)) && ros::ok()) {
		ROS_WARN("Trying to register to robot_manager/unload_nodelet...");
	}
	_unloadRobotClient = _nh.serviceClient<nodelet::NodeletUnload>("robot_manager/unload_nodelet");
	
//~ 	_registerRobotServer.registerGoalCallback( boost::bind(&Server::registerRobotCallback, this) );
//~ 	_registerRobotServer.start();	
}

bool Server::loadMapCallback(stdr_msgs::LoadMap::Request& req,
							stdr_msgs::LoadMap::Response& res) 
{
	if (_mapServer) {
		ROS_WARN("Map already loaded!");
		return false;
	}
	_mapServer.reset(new MapServer(req.mapFile));
	// if we don't have map, no point to spawn robot
	_spawnRobotServer.start();

	return true;	
}

void Server::spawnRobotCallback() {
	stdr_msgs::RobotMsg description = _spawnRobotServer.acceptNewGoal()->description;
	
	addNewRobot(description);
}


void Server::addNewRobot(stdr_msgs::RobotMsg description) {
	
	stdr_msgs::RobotIndexedMsg namedRobot;
	
	namedRobot.robot = description;
	namedRobot.name = "/robot" + boost::lexical_cast<std::string>(_id++);
	
	_robotMap.insert( std::make_pair(namedRobot.name, namedRobot) );
	
	nodelet::NodeletLoad srv;
	srv.request.name = namedRobot.name;
	srv.request.type = "stdr_robot/Robot";
	
	// add contitional variable to wait from register robot
	
	if (_spawnRobotClient.call(srv)) {
		stdr_msgs::SpawnRobotResult result;
		result.indexedDescription = namedRobot;
		_spawnRobotServer.setSucceeded(result);
	}
	
	_spawnRobotServer.setAborted();
}

}
