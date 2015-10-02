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
#include <stdr_msgs/KinematicMsg.h>

#include <ctime>

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
      @brief Virtual function - Callback for velocity commands
      @param msg [const geometry_msgs::Twist&] The velocity command
      @return void
      **/
      virtual void velocityCallback(const geometry_msgs::Twist& msg)
      {
        _currentTwist = msg;
        sampleVelocities();
      }

      /**
      @brief Virtual function - Add noise to velocity commands
      @param msg [geometry_msgs::Twist&] The velocity command
      @return void
      **/
      /**     
      The formulas used are:
      - u_x' = u_x + Sample(a_ux_ux * u_x^2 + a_ux_uy * u_y^2 + a_ux_w * w^2)
      - u_y' = u_y + Sample(a_uy_ux * u_x^2 + a_uy_uy * u_y^2 + a_uy_w * w^2)
      - w' = w + Sample(a_w_ux * u_x^2 + a_w_uy * u_y^2 + a_w_w * w^2)
      - g' = Sample(a_g_ux * u_x^2 + a_g_uy * u_y^2 + a_g_w * w^2)
      Then w' is used as such (depending on the model):
      theta' = theta + (w' + g') * Dt
      Sample(b^2) produces samples from a normal distribution with variance
      equal to b^2.
      **/
      virtual void sampleVelocities(void)
      {
        float ux = _currentTwist.linear.x;
        float uy = _currentTwist.linear.y;
        float w = _currentTwist.angular.z;

        float sample_ux = 
          _motion_parameters.a_ux_ux * ux * ux +
          _motion_parameters.a_ux_uy * uy * uy +
          _motion_parameters.a_ux_w  * w  * w;
        _currentTwist.linear.x += sampleNormal(sqrt(sample_ux));

        float sample_uy = 
          _motion_parameters.a_uy_ux * ux * ux +
          _motion_parameters.a_uy_uy * uy * uy +
          _motion_parameters.a_uy_w  * w  * w;
        _currentTwist.linear.y += sampleNormal(sqrt(sample_uy));
 
        float sample_w = 
          _motion_parameters.a_w_ux * ux * ux +
          _motion_parameters.a_w_uy * uy * uy +
          _motion_parameters.a_w_w  * w  * w;
        _currentTwist.angular.z += sampleNormal(sqrt(sample_w));

        float sample_g = 
          _motion_parameters.a_g_ux * ux * ux +
          _motion_parameters.a_g_uy * uy * uy +
          _motion_parameters.a_g_w  * w  * w;
        _currentTwist.angular.z += sampleNormal(sqrt(sample_g));
      }

      
      /**
      @brief Virtual function - Stops the robot
      @return void
      **/
      virtual void stop(void)
      {
        _currentTwist.linear.x = 0;
        _currentTwist.linear.y = 0;
        _currentTwist.linear.z = 0;
        _currentTwist.angular.x = 0;
        _currentTwist.angular.y = 0;
        _currentTwist.angular.z = 0;
      }
      
      /**
      @brief Pure virtual function - Calculates the motion - updates the robot pose
      @param event [const ros::TimerEvent&] A ROS timer event
      @return void
      **/
      virtual void calculateMotion(const ros::TimerEvent& event) = 0;
      
      /**
      @brief Returns the pose calculated by the motion controller
      @return geometry_msgs::Pose2D
      **/
      inline geometry_msgs::Pose2D getPose(void)
      {
        return _pose;
      }
      
      /**
      @brief Sets the initial pose of the motion controller
      @param new_pose [geometry_msgs::Pose2D] The new pose
      @return void
      **/
      inline void setPose(geometry_msgs::Pose2D new_pose)
      {
        _pose.x = new_pose.x;
        _pose.y = new_pose.y;
        _pose.theta = new_pose.theta;
      }
      
      /**
      @brief Get the current velocity of the motion controller
      @return geometry_msgs::Twist
      */
      inline geometry_msgs::Twist getVelocity() {
        return _currentTwist;
      }

      /**
      @brief Default desctructor
      @return void
      **/
      virtual ~MotionController(void) 
      {
      }

      /**
      @brief Approaches a normal distribution sampling
      @return float
      source: Sebastian Thrun, Probabilistic Robotics
      **/
      float sampleNormal(float sigma)
      {
        float tmp = 0;
        for (unsigned int i = 0 ; i < 12 ; i++)
        {
          float sample = (rand() % 100000) / 50000.0 - 1.0; // From -1.0 -> 1.0
          tmp += sample * sigma;
        }
        return tmp / 2.0;
      }

    
    protected:
      
      /**
      @brief Default constructor
      @param pose [const geometry_msgs::Pose2D&] The robot pose
      @param tf [tf::TransformBroadcaster&] A ROS tf broadcaster
      @param name [const std::string&] The robot frame id
      @return void
      **/
      MotionController(
        const geometry_msgs::Pose2D& pose, 
        tf::TransformBroadcaster& tf, 
        const std::string& name,
        ros::NodeHandle& n,
        const stdr_msgs::KinematicMsg params
        )
          : _tfBroadcaster(tf), 
            _freq(0.1), 
            _namespace(name),
            _pose(pose),
            _motion_parameters(params)
        { 
          _velocitySubscrider = n.subscribe(
            _namespace + "/cmd_vel",
            1,
            &MotionController::velocityCallback,
            this);  
 
          srand(time(NULL));
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
      geometry_msgs::Pose2D _pose;
      //!< Current motion command
      geometry_msgs::Twist _currentTwist;
      //!< The kinematic model parameters
      stdr_msgs::KinematicMsg _motion_parameters;
  };
    
  typedef boost::shared_ptr<MotionController> MotionControllerPtr;
    
}


#endif
