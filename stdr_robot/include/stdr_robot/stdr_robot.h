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
#include <stdr_robot/sensors/laser.h>
#include <stdr_robot/sensors/sonar.h>
#include <stdr_robot/sensors/rfid_reader.h>
#include <stdr_robot/sensors/co2.h>
#include <stdr_robot/sensors/microphone.h>
#include <stdr_robot/sensors/thermal.h>
#include <stdr_robot/motion/motion_controller_base.h>
#include <stdr_robot/motion/ideal_motion_controller.h>
#include <stdr_robot/motion/omni_motion_controller.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <stdr_msgs/RegisterRobotAction.h>

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {

  typedef actionlib::SimpleActionClient<stdr_msgs::RegisterRobotAction> 
    RegisterRobotClient;
  typedef boost::shared_ptr<RegisterRobotClient> RegisterRobotClientPtr;

  /**
  @class Robot
  @brief Represents one robot in STDR. Inherts publicly from nodelet::Nodelet
  **/ 
  class Robot : public nodelet::Nodelet {
    
   public: 
    
    /**
    @brief Default constructor
    @return void
    **/
    Robot(void);
    
    /**
    @brief Initializes the robot and gets the environment occupancy grid map
    @return void
    **/
    void onInit(void);
    
    /**
    @brief Initializes the robot after on registering it to server
    @param state [const actionlib::SimpleClientGoalState&] State of action
    @param result [const stdr_msgs::RegisterRobotResultConstPtr] Action result of registering the robot
    @return void
    **/
    void initializeRobot(const actionlib::SimpleClientGoalState& state, 
      const stdr_msgs::RegisterRobotResultConstPtr result);
      
    /**
    @brief Callback for getting the occupancy grid map
    @param msg [const nav_msgs::OccupancyGridConstPtr&] The occupancy grid map
    @return void
    **/
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    
    /**
    @brief The callback of the re-place robot service
    @param req [stdr_msgs::MoveRobot::Request&] The service request
    @param res [stdr_msgs::MoveRobot::Response&] The service result
    @return bool
    **/
    bool moveRobotCallback(stdr_msgs::MoveRobot::Request& req,
      stdr_msgs::MoveRobot::Response& res);
      
    /**
    @brief Default destructor
    @return void
    **/
    ~Robot(void);  
    
   private:
   
    /**
    @brief Checks the robot collision -2b changed-
    @return void
    **/
    bool collisionExists(
      const geometry_msgs::Pose2D& newPose, 
      const geometry_msgs::Pose2D& collisionPoint);
      
    bool collisionExistsNoPath(
      const geometry_msgs::Pose2D& newPose); 

    /**
    @brief Checks the robot's reposition into unknown area
    @param newPose [const geometry_msgs::Pose2D] The pose for the robot to be moved to
    @return True when position is in unknown area
     y**/
    bool checkUnknownOccupancy(const geometry_msgs::Pose2D& newPose);

    /**
    @brief Publishes the tf transforms every with 10Hz
    @return void
    **/
    void publishTransforms(const ros::TimerEvent&);
   
   
   private:
  
    //!< ROS subscriber for map
    ros::Subscriber _mapSubscriber;
    
    //!< ROS timer to publish tf transforms (10Hz)
    ros::Timer _tfTimer;
    
    //!< ROS service server to move robot
    ros::ServiceServer _moveRobotService;
  
    //!< Container for robot sensors
    SensorPtrVector _sensors;
    
    //!< The occupancy grid map
    nav_msgs::OccupancyGrid _map;
    
    //!< ROS tf transform broadcaster
    tf::TransformBroadcaster _tfBroadcaster;

    //!< Odometry Publisher
    ros::Publisher _odomPublisher;
    
    //!< Holds robots current pose
    geometry_msgs::Pose2D _currentPose;
    
    //!< Holds robots previous pose
    geometry_msgs::Pose2D _previousPose;
    
    //!< Pointer of a motion controller
    MotionControllerPtr _motionControllerPtr;
    
    //!< Actionlib client for registering the robot
    RegisterRobotClientPtr _registerClientPtr;
  
    //!< The robot footprint in points (row * 10000 + col)
    std::vector<std::pair<float,float> > _footprint;
    
    std::vector<std::pair<int,int> > getPointsBetween(
      int x1, int y1, int x2, int y2) ;
    
    //!< Robot's previous movement direction in X Axis
    bool _previousMovementXAxis;

    //!< Robot's previous movement direction in Y Axis
    bool _previousMovementYAxis;
  };  
  
} // namespace stdr_robot


#endif
