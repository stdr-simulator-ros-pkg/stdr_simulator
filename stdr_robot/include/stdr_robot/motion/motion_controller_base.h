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

#ifndef MOTION_CONTROLLER_BASE_H
#define MOTION_CONTROLLER_BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {
    
  /**
  @class MotionController
  @brief Abstract class that provides motion controller abstraction
  **/ 
  class MotionController {
    
    public:
      
      /**
      @brief Pure virtual function - Callback for velocity commands
      @param msg [const geometry_msgs::Twist&] The velocity command
      @return void
      **/
      virtual void velocityCallback(const geometry_msgs::Twist& msg) = 0;
      
      /**
      @brief Pure virtual function - Stops the robot
      @return void
      **/
      virtual void stop(void) = 0;
      
      /**
      @brief Pure virtual function - Calculates the motion - updates the robot pose
      @param event [const ros::TimerEvent&] A ROS timer event
      @return void
      **/
      virtual void calculateMotion(const ros::TimerEvent& event) = 0;
      
      /**
      @brief Default desctructor
      @return void
      **/
      virtual ~MotionController(void) 
      {
      }
    
    protected:
      
      /**
      @brief Default constructor
      @param pose [const geometry_msgs::Pose2DPtr&] The robot pose
      @param tf [tf::TransformBroadcaster&] A ROS tf broadcaster
      @param name [const std::string&] The robot frame id
      @return void
      **/
      MotionController(
        const geometry_msgs::Pose2DPtr& pose, 
        tf::TransformBroadcaster& tf, 
        const std::string& name)
          : _posePtr(pose), _tfBroadcaster(tf), _freq(0.1), _namespace(name) 
      { 
      }
    
    protected:
      
      //!< The base of the frame_id
      const std::string& _namespace;
      //!< ROS subscriber to the velocity topic
      ros::Subscriber _velocitySubscrider;
      //!< Frequency of motion calculation
      ros::Duration _freq;
      //!< ROS timer for generating motion calculation events
      ros::Timer _calcTimer;
      //!< Broadcaster of the robot tf transform
      tf::TransformBroadcaster& _tfBroadcaster;
      //!< Robot pose message
      const geometry_msgs::Pose2DPtr& _posePtr;
      //!< Current motion command
      geometry_msgs::Twist _currentTwist;
  };
    
  typedef boost::shared_ptr<MotionController> MotionControllerPtr;
    
}


#endif
