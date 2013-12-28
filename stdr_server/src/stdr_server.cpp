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
  : _spawnRobotServer(_nh, "stdr_server/spawn_robot", boost::bind(&Server::spawnRobotCallback, this, _1), false)
  , _registerRobotServer(_nh, "stdr_server/register_robot", boost::bind(&Server::registerRobotCallback, this, _1), false)
  , _deleteRobotServer(_nh, "stdr_server/delete_robot", boost::bind(&Server::deleteRobotCallback, this, _1), false)
  , _id(0)
{
//~ 	_spawnRobotServer.registerGoalCallback( boost::bind(&Server::spawnRobotCallback, this) );
	
	if (argc > 2) {
		ROS_ERROR("%s", USAGE);
		exit(-1);
	}
	
	if (argc == 2) {
		std::string fname(argv[1]);
		_mapServer.reset(new MapServer(fname));
		// if we don't have map, no point to start servers
		activateActionServers();
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
	
	_robotsPublisher = _nh.advertise<stdr_msgs::RobotIndexedVectorMsg>("stdr_server/active_robots", 10, true);
}

bool Server::loadMapCallback(stdr_msgs::LoadMap::Request& req,
							stdr_msgs::LoadMap::Response& res) 
{
	if (_mapServer) {
		ROS_WARN("Map already loaded!");
		return false;
	}
	_mapServer.reset(new MapServer(req.mapFile));
	// if we don't have map, no point to start servers
	activateActionServers();

	return true;	
}

bool Server::loadExternalMapCallback(stdr_msgs::LoadExternalMap::Request& req,
							stdr_msgs::LoadExternalMap& res)
{
	if (_mapServer) {
		ROS_WARN("Map already loaded!");
		return false;
	}
	_mapServer.reset(new MapServer(req.map));
	// if we don't have map, no point to start servers
	activateActionServers();

	return true;	
}

void Server::spawnRobotCallback(const stdr_msgs::SpawnRobotGoalConstPtr& goal) {
	
	stdr_msgs::SpawnRobotResult result;
	
	if (addNewRobot(goal->description, &result)) {
		_spawnRobotServer.setSucceeded(result);
		
		// publish to active_robots topic
		stdr_msgs::RobotIndexedVectorMsg msg;
		for (RobotMap::iterator it=_robotMap.begin(); it!=_robotMap.end(); ++it) {
			msg.robots.push_back( it->second );
		}
		
		_robotsPublisher.publish(msg);
		return;
	}
	
	_spawnRobotServer.setAborted();
}

void Server::deleteRobotCallback(const stdr_msgs::DeleteRobotGoalConstPtr&  goal) {
	
	stdr_msgs::DeleteRobotResult result;
	
	if (deleteRobot(goal->name, &result)) {
		
		// publish to active_robots topic
		stdr_msgs::RobotIndexedVectorMsg msg;
		for (RobotMap::iterator it=_robotMap.begin(); it!=_robotMap.end(); ++it) {
			msg.robots.push_back( it->second );
		}
		_robotsPublisher.publish(msg);
		_deleteRobotServer.setSucceeded(result);
		return;
	}
	
	_deleteRobotServer.setAborted(result);
}

void Server::registerRobotCallback(const stdr_msgs::RegisterRobotGoalConstPtr& goal) {
	
	boost::unique_lock<boost::mutex> lock(_mut);
	stdr_msgs::RegisterRobotResult result;
	result.description = _robotMap[goal->name].robot;
	_registerRobotServer.setSucceeded(result);
	// notify spawn action, to reply to spawnner
	cond.notify_one();
}


void Server::activateActionServers() {
	_spawnRobotServer.start();
	_registerRobotServer.start();
	_deleteRobotServer.start();
}

bool Server::addNewRobot(stdr_msgs::RobotMsg description, stdr_msgs::SpawnRobotResult* result) {
	
	stdr_msgs::RobotIndexedMsg namedRobot;
	
	namedRobot.robot = description;
	namedRobot.name = "/robot" + boost::lexical_cast<std::string>(_id++);
	
	_robotMap.insert( std::make_pair(namedRobot.name, namedRobot) );
	
	nodelet::NodeletLoad srv;
	srv.request.name = namedRobot.name;
	srv.request.type = "stdr_robot/Robot";
	
	boost::unique_lock<boost::mutex> lock(_mut);
		
	if (_spawnRobotClient.call(srv)) {
		// wait until robot calls RobotRegisterAction
		cond.wait(lock);
		
		result->indexedDescription = namedRobot;
		
		lock.unlock();
		return true;
	}
	
	lock.unlock();
	return false;
}

bool Server::deleteRobot(std::string name, stdr_msgs::DeleteRobotResult* result) {
	
	RobotMap::iterator it = _robotMap.find(name);
	
	if (it != _robotMap.end()) {
		
		nodelet::NodeletUnload srv;
		srv.request.name =  name;
		
		if (_unloadRobotClient.call(srv)) {
			
			if (srv.response.success) {
				_robotMap.erase(it);
			}
			
			result->success = srv.response.success;
			return srv.response.success;
		}
		
		result->success = false;
		return false;
	}
	
	ROS_WARN("Requested to delete robot, with name %s does not exist.", name.c_str());
	
	result->success = false;
	return false;	
}

} // end of namespace stdr_robot
