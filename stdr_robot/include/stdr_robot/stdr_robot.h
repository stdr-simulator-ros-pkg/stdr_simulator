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

#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_broadcaster.h>
#include <stdr_msgs/RobotMsg.h>
#include <stdr_msgs/MoveRobot.h>
#include <stdr_robot/sensors/sensor_base.h>
#include <stdr_robot/motion/motion_controller_base.h>
#include <stdr_robot/motion/ideal_motion_controller.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <stdr_msgs/RegisterRobotAction.h>


namespace stdr_robot {

typedef actionlib::SimpleActionClient<stdr_msgs::RegisterRobotAction> RegisterRobotClient;
typedef boost::shared_ptr<RegisterRobotClient> RegisterRobotClientPtr;

class Robot : public nodelet::Nodelet {
	
	public: 
		
		Robot() {}
		void onInit();
		void initializeRobot(const actionlib::SimpleClientGoalState& state, const stdr_msgs::RegisterRobotResultConstPtr result);
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
		bool moveRobotCallback(stdr_msgs::MoveRobot::Request& req,
							stdr_msgs::MoveRobot::Response& res);
		~Robot();
		
	private:
		
		void checkCollision(const ros::TimerEvent&);
		void publishRobotTf(const ros::TimerEvent&);
	
	private:
	
		ros::Subscriber _mapSubscriber;
		ros::Subscriber _descriptionSubscriber;
		ros::Timer _collisionTimer;
		ros::Timer _tfTimer;
		ros::ServiceServer _moveRobotService;
	
		SensorPtrVector _sensors;
		nav_msgs::OccupancyGrid _map;
		tf::TransformBroadcaster _tfBroadcaster;
		geometry_msgs::Pose2D _currentPose;
		MotionControllerPtr _motionControllerPtr;
		
		RegisterRobotClientPtr _registerClientPtr;
};	
	
}


#endif
