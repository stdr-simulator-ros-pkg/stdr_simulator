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

namespace stdr_robot {

  /**
  @brief Default constructor
  @return void
  **/
  HandleRobot::HandleRobot() 
    : _spawnRobotClient("stdr_server/spawn_robot", true)
    , _deleteRobotClient("stdr_server/delete_robot", true)
  {
  }

  /**
  @brief Spawns a new robot
  @param msg [const stdr_msgs::RobotMsg] The robot message from which the robot is created
  @return stdr_msgs::RobotIndexedMsg : The robot message with the proper frame_id
  **/
  stdr_msgs::RobotIndexedMsg HandleRobot::spawnNewRobot(
    const stdr_msgs::RobotMsg msg) 
  {
    
    stdr_msgs::SpawnRobotGoal goal;
    goal.description = msg;
      
    while (!_spawnRobotClient.waitForServer(ros::Duration(1)) && ros::ok()) {
      ROS_WARN("Could not find stdr_server/spawn_robot action.");
    }
    
    _spawnRobotClient.sendGoal(goal);
    
    bool success = _spawnRobotClient.waitForResult(ros::Duration(10));
    if (!success) {
      throw ConnectionException("Could not spawn robot...");
    }
    
    actionlib::SimpleClientGoalState state = _spawnRobotClient.getState();
    if(state.toString() == "ABORTED")
    {
      std::string msg = std::string("Could not spawn robot. ") + 
        _spawnRobotClient.getResult()->message;
      throw DoubleFrameIdException(msg);
    }
    
    ROS_INFO("New robot spawned successfully, with name %s.", 
      _spawnRobotClient.getResult()->indexedDescription.name.c_str());
    
    return _spawnRobotClient.getResult()->indexedDescription;
    
  }

  /**
  @brief Deletes a robot by frame_id
  @param name [const std::string&] The robot frame_id to be deleted
  @return bool : True if deletion was successful
  **/
  bool HandleRobot::deleteRobot(const std::string& name) 
  {
    
    stdr_msgs::DeleteRobotGoal goal;
    goal.name = name;
    
    while (!_deleteRobotClient.waitForServer(ros::Duration(1)) && ros::ok()) {
      ROS_WARN("Could not find stdr_server/delete_robot action.");
    }
    
    _deleteRobotClient.sendGoal(goal);
    
    bool success = _deleteRobotClient.waitForResult(ros::Duration(10));
    
    if (!success) {
      throw ConnectionException("Could not delete robot, connection error...");
    }
    
    return _deleteRobotClient.getResult()->success;
    
  }

  /**
  @brief Re-places a robot by frame_id
  @param name [const std::string&] The robot frame_id to be moved
  @param newPose [const geometry_msgs::Pose2D] The pose for the robot to be moved to
  @return bool : True if move was successful
  **/
  bool HandleRobot::moveRobot(
    const std::string& name, const geometry_msgs::Pose2D newPose) 
  {
    
    while (!ros::service::waitForService(name + "/replace", 
      ros::Duration(.1)) && ros::ok()) {
      ROS_WARN("Could not find %s/replace ...", name.c_str());
    }
    
    stdr_msgs::MoveRobot srv;
    srv.request.newPose = newPose;
    
    if (ros::service::call(name + "/replace", srv)) {
      return true;
    }
    
    return false;
  }
    
} // end of namespace stdr_robot
