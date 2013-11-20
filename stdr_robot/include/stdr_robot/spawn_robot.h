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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <stdr_msgs/RobotIndexedMsg.h>
#include <stdr_msgs/SpawnRobotAction.h>
#include <stdr_robot/exceptions.h>

#ifndef SPAWN_ROBOT_H
#define SPAWN_ROBOT_H

namespace stdr_robot {

typedef actionlib::SimpleActionClient<stdr_msgs::SpawnRobotAction> SpawnRobotClient;

class SpawnRobot {
	
	public:
		
		SpawnRobot();
		stdr_msgs::RobotIndexedMsg spawnNewRobot(const stdr_msgs::RobotMsg msg);
		
	private:
		
		SpawnRobotClient _spawnRobotClient;
		
};
	
}

#endif
