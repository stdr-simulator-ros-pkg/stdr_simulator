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
   * Oscar Lima, olima_84@yahoo.com - omnidirectional controller part (lines 88-125)
   * Ashok Meenakshi, mashoksc@gmail.com - omnidirectional controller part (lines 88-125)
******************************************************************************/

#include <stdr_robot/motion/ideal_motion_controller.h>
#include <stdr_robot/motion/omni_motion_controller.h>
#include <stdr_robot/motion/pose.h>

namespace stdr_robot {
    
  /**
  @brief Default constructor
  @param pose [const geometry_msgs::Pose2D&] The robot pose
  @param tf [tf::TransformBroadcaster&] A ROS tf broadcaster
  @param n [ros::NodeHandle&] The ROS node handle
  @param name [const std::string&] The robot frame id
  @return void
  **/
  IdealMotionController::IdealMotionController(
    const geometry_msgs::Pose2D& pose, 
    tf::TransformBroadcaster& tf, 
    ros::NodeHandle& n, 
    const std::string& name)
      : MotionController(pose, tf, name)
  {
    _velocitySubscrider = n.subscribe(
      _namespace + "/cmd_vel", 
      1, 
      &IdealMotionController::velocityCallback, 
      this);
    
    _calcTimer = n.createTimer(
      _freq, 
      &IdealMotionController::calculateMotion, 
      this);
  }

  /**
  @brief Callback for velocity commands
  @param msg [const geometry_msgs::Twist&] The velocity command
  @return void
  **/
  void IdealMotionController::velocityCallback(const geometry_msgs::Twist& msg) 
  {
    _currentTwist = msg;
  }
    
  /**
  @brief Stops the robot
  @return void
  **/
  void IdealMotionController::stop() 
  {
    _currentTwist.linear.x = 0;
	_currentTwist.linear.y = 0; //line added for omni controller
    _currentTwist.angular.z = 0;
  }

  /**
  @brief Calculates the motion - updates the robot pose
  @param event [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void IdealMotionController::calculateMotion(const ros::TimerEvent& event) 
  {
	  
		//!< updates _posePtr based on _currentTwist and time passed (event.last_real)
		
		//omni controller
		
		//computing delta time
		ros::Duration dt = ros::Time::now() - event.last_real;
		
		//if speed is zero then do nothing
		if (!(_currentTwist.linear.x == 0 && _currentTwist.linear.y == 0 && _currentTwist.angular.z == 0)) 
		{
			//Create motion model with 0.0 mean error and 0.0 std deviation
			MotionModel youbot_model(0.0, 0.0);
			
			//declaring linear and angular velocities
			float vx = _currentTwist.linear.x;
			float vy = _currentTwist.linear.y;
			float vtheta = _currentTwist.angular.z;
			
			//computing linear and angular displacement, assuming constant acceleration
			float dx = vx * dt.toSec();
			float dy = vy * dt.toSec();
			float dtheta = vtheta * dt.toSec();
			
			//set displacement in the motion model
			youbot_model.setMotion(dx, dy, dtheta);
			
			//sample the motion model based on the displacemet and current pose
			Pose current_pose;
			current_pose.x = _pose.x;
			current_pose.y = _pose.y;
			current_pose.theta = _pose.theta;
			Pose new_pose = youbot_model.sample(current_pose);
			
			//updating x,y and theta position
			_pose.x = new_pose.x;
			_pose.y = new_pose.y;
			_pose.theta = new_pose.theta;
		}
  }
  
  /**
  @brief Default destructor 
  @return void
  **/
  IdealMotionController::~IdealMotionController(void)
  {
    
  }
    
}
