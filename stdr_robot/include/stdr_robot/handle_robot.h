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
#include <actionlib/client/terminal_state.h>
#include <stdr_msgs/RobotIndexedMsg.h>
#include <stdr_msgs/SpawnRobotAction.h>
#include <stdr_msgs/DeleteRobotAction.h>
#include <stdr_msgs/MoveRobot.h>
#include <stdr_robot/exceptions.h>
#include <geometry_msgs/Pose2D.h>

#ifndef HANDLE_ROBOT_H
#define HANDLE_ROBOT_H

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {

  typedef actionlib::SimpleActionClient<stdr_msgs::SpawnRobotAction> 
    SpawnRobotClient;
  typedef actionlib::SimpleActionClient<stdr_msgs::DeleteRobotAction> 
    DeleteRobotClient;

  /**
  @class HandleRobot
  @brief Handles the manipulation of robots (creation, deletion, move)
  **/ 
  class HandleRobot {
    //------------------------------------------------------------------------//
    private:
    
      //!< Action client for spawning robots
      SpawnRobotClient _spawnRobotClient;
      //!< Action client for deleting robots
      DeleteRobotClient _deleteRobotClient;
      
    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default constructor
      @return void
      **/
      HandleRobot();
      
      /**
      @brief Spawns a new robot
      @param msg [const stdr_msgs::RobotMsg] The robot message from which the robot is created
      @return stdr_msgs::RobotIndexedMsg : The robot message with the proper frame_id
      **/
      stdr_msgs::RobotIndexedMsg spawnNewRobot(const stdr_msgs::RobotMsg msg);
      
      /**
      @brief Deletes a robot by frame_id
      @param name [const std::string&] The robot frame_id to be deleted
      @return bool : True if deletion was successful
      **/
      bool deleteRobot(const std::string& name);
      
      /**
      @brief Re-places a robot by frame_id
      @param name [const std::string&] The robot frame_id to be moved
      @param newPose [const geometry_msgs::Pose2D] The pose for the robot to be moved to
      @return bool : True if move was successful
      **/
      bool moveRobot(const std::string& name, 
        const geometry_msgs::Pose2D newPose);
  };
}

#endif
