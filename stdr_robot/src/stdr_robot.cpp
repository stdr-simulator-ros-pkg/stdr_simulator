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

#include <stdr_robot/stdr_robot.h>
#include <nodelet/NodeletUnload.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(stdr_robot::Robot, nodelet::Nodelet) 

namespace stdr_robot {
	
void Robot::onInit() {
	ros::NodeHandle n = getMTNodeHandle();
	
	_registerClientPtr.reset( new RegisterRobotClient(n, "stdr_server/register_robot", true) );
	_registerClientPtr->waitForServer();
	
	stdr_msgs::RegisterRobotGoal goal;
	goal.name = getName();
	_registerClientPtr->sendGoal(goal, boost::bind(&Robot::initializeRobot, this, _1, _2));	

	_motionControllerPtr.reset( new IdealMotionController(geometry_msgs::Pose2DPtr(&_currentPose), _tfBroadcaster, n, getName()) );
	_mapSubscriber = n.subscribe("map", 1, &Robot::mapCallback, this);
	_collisionTimer = n.createTimer(ros::Duration(0.1), &Robot::checkCollision, this);
}

void Robot::initializeRobot(const actionlib::SimpleClientGoalState& state, const stdr_msgs::RegisterRobotResultConstPtr result) {
	
	if (state == state.ABORTED) {
		NODELET_ERROR("Something really bad happened...");
		return;
	}
	
	NODELET_INFO("Loaded new robot, %s", getName().c_str());
	
	// use result to initialize sensors

}

void Robot::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
	_map = *msg;
}

void Robot::checkCollision(const ros::TimerEvent&) {
	// check if we have a collision and notify MotionController via stop() interface
}

Robot::~Robot() {
	// cleanup
}
	
}
