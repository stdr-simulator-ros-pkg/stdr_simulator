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

#include <stdr_robot/motion/ideal_motion_controller.h>

namespace stdr_robot {
    
  /**
  @brief Default constructor
  @param pose [const geometry_msgs::Pose2DPtr&] The robot pose
  @param tf [tf::TransformBroadcaster&] A ROS tf broadcaster
  @param n [ros::NodeHandle&] The ROS node handle
  @param name [const std::string&] The robot frame id
  @return void
  **/
  IdealMotionController::IdealMotionController(
    const geometry_msgs::Pose2DPtr& pose, 
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
    
    ros::Duration dt = ros::Time::now() - event.last_real;
    
    if (_currentTwist.angular.z == 0) {
      
      _posePtr->x += _currentTwist.linear.x*dt.toSec()*cosf(_posePtr->theta);
      _posePtr->y += _currentTwist.linear.x*dt.toSec()*sinf(_posePtr->theta);
    }
    else {
      
      _posePtr->x += - _currentTwist.linear.x / _currentTwist.angular.z * 
        sinf(_posePtr->theta) + 
        _currentTwist.linear.x / _currentTwist.angular.z * 
        sinf(_posePtr->theta + dt.toSec() * _currentTwist.angular.z);
      
      _posePtr->y -= - _currentTwist.linear.x / _currentTwist.angular.z * 
        cosf(_posePtr->theta) + 
        _currentTwist.linear.x / _currentTwist.angular.z * 
        cosf(_posePtr->theta + dt.toSec() * _currentTwist.angular.z);
    }
    _posePtr->theta += _currentTwist.angular.z*dt.toSec();
    
  }
  
  /**
  @brief Default destructor 
  @return void
  **/
  IdealMotionController::~IdealMotionController(void)
  {
    
  }
    
}
