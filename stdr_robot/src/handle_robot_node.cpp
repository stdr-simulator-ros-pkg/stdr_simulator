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
#include <stdr_parser/stdr_parser.h>

#define USAGE "USAGE: robot_handler add <description.yaml> <x> <y> <theta>\n" \
"OR: robot_handler delete <robot_name>\n"\
"OR: robot_handler replace <robot_name> <new_x> <new_y> <new_theta>"

/**
@brief Main function of the server node
@param argc [int] Number of input arguments
@param argv [char**] Input arguments
@return int
**/
int main(int argc, char** argv) {
  
  ros::init(argc, argv, "robot_spawner", ros::init_options::AnonymousName);
  
  stdr_robot::HandleRobot handler;
  
  //!< add
  if (((argc == 3) || (argc == 6)) && (std::string(argv[1]) == "add")) {
    
    stdr_msgs::RobotMsg msg;
    
    try {
      msg = stdr_parser::Parser::createMessage
        <stdr_msgs::RobotMsg>(std::string(argv[2]));
    }
    catch(stdr_parser::ParserException& ex)
    {
      ROS_ERROR("[STDR_PARSER] %s", ex.what());
      return -1;
    }
    
    if (argc == 6) {
      msg.initialPose.x = atof(argv[3]);
      msg.initialPose.y = atof(argv[4]);
      msg.initialPose.theta = atof(argv[5]);
    }
    
    stdr_msgs::RobotIndexedMsg namedRobot;
    
    try {
      namedRobot = handler.spawnNewRobot(msg);
      return 0;
    }
    catch (stdr_robot::ConnectionException& ex) {
      ROS_ERROR("%s", ex.what());
      return -1;
    }
    
  }
  //!< delete
  else if ((argc == 3) && (std::string(argv[1]) == "delete")) {
    
    std::string name(argv[2]);
    
    try {
      if (handler.deleteRobot(name)) {
        ROS_INFO("Robot %s deleted successfully", name.c_str());
      }
      else {
        ROS_ERROR("Could not delete robot %s", name.c_str());
      }
      
      return 0;
    }
    catch (stdr_robot::ConnectionException& ex) {
      ROS_ERROR("%s", ex.what());
      return -1;
    }
    
  }
  
  //!< replacement
  else if ((argc == 6) && (std::string(argv[1]) == "replace")) {
    
    std::string name(argv[2]);
    
    geometry_msgs::Pose2D newPose;
    newPose.x = atof(argv[3]);
    newPose.y = atof(argv[4]);
    newPose.theta = atof(argv[5]);
    
    if (handler.moveRobot(name, newPose)) {
      ROS_INFO("%s moved to new pose with x: %f, y: %f, theta: %f", 
        name.c_str(), newPose.x, newPose.y, newPose.theta);
      return 0;
    }
    
    ROS_ERROR("Could not move %s", name.c_str());
    return -1;
  }
  // wrong args
  else {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
}
