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

#include <stdr_robot/handle_robot.h>

#define USAGE "USAGE: robot_handler [add|delete|move]\n" \
              "  map.yaml: map description file\n" 

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "robot_spawner", ros::init_options::AnonymousName);
	
	stdr_robot::HandleRobot handler;
	
	// add
	if ((argc == 2) && (std::string(argv[1]) == "add")) {
		
		stdr_msgs::RobotMsg msg;
	
		stdr_msgs::RobotIndexedMsg namedRobot;
		
		try {
			namedRobot = handler.spawnNewRobot(msg);
			return 0;
		}
		catch (ConnectionException& ex) {
			ROS_ERROR("%s", ex.what());
			return -1;
		}
		
	}
	// delete
	else if ((argc == 3) && (std::string(argv[1]) == "delete")) {
		
		std::string name(argv[2]);
		
		try {
			if (handler.deleteRobot(name)) {
				ROS_INFO("Robot %s deleted successfully", name.c_str());
			}
			else {
				ROS_ERROR("Could not delete robot %s", name.c_str());
			}
			
			return 0;
		}
		catch (ConnectionException& ex) {
			ROS_ERROR("%s", ex.what());
			return -1;
		}
		
	}
	// wrong args
	else {
		ROS_ERROR("%s", USAGE);
		exit(-1);
	}
}
