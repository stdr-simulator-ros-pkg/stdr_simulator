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

#ifndef STDR_SERVER_H
#define STDR_SERVER_H

#define USAGE "\nUSAGE: stdr_server <map.yaml>\n" \
              "  map.yaml: map description file\n" 

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <stdr_server/map_server.h>
#include <stdr_msgs/LoadMap.h>
#include <stdr_msgs/LoadExternalMap.h>
#include <stdr_msgs/RegisterGui.h>
#include <stdr_msgs/RegisterRobotAction.h>
#include <stdr_msgs/SpawnRobotAction.h>
#include <stdr_msgs/DeleteRobotAction.h>
#include <stdr_msgs/RobotIndexedMsg.h>
#include <stdr_msgs/RobotIndexedVectorMsg.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>

/**
@namespace stdr_server
@brief The main namespace for STDR Server
**/ 
namespace stdr_server {

  typedef boost::shared_ptr<MapServer> MapServerPtr;
  
  typedef actionlib::SimpleActionServer<stdr_msgs::SpawnRobotAction> 
    SpawnRobotServer;
  
  typedef actionlib::SimpleActionServer<stdr_msgs::RegisterRobotAction> 
    RegisterRobotServer;
  
  typedef actionlib::SimpleActionServer<stdr_msgs::DeleteRobotAction> 
    DeleteRobotServer;

  typedef std::map<std::string, stdr_msgs::RobotIndexedMsg> RobotMap;

  /**
  @class Server
  @brief Implements the STDR server functionalities
  **/ 
  class Server 
  {
      
    public:
      
      /**
      @brief Default constructor
      @param argc [int] Number of input arguments
      @param argv [char**] The input arguments
      @return void
      **/
      Server(int argc, char** argv);
      
      //!< Services --------------------------
      
      /**
      @brief Service callback for loading the map
      @param req [stdr_msgs::LoadMap::Request&] The service request
      @param res [stdr_msgs::LoadMap::Response&] The service response
      @return bool
      **/
      bool loadMapCallback(stdr_msgs::LoadMap::Request& req,
        stdr_msgs::LoadMap::Response& res);
      
      /**
      @brief Service callback for loading the map from GUI
      @param req [stdr_msgs::LoadExternalMap::Request&] The service request
      @param res [stdr_msgs::LoadExternalMap::Response&] The service response
      @return bool
      **/
      bool loadExternalMapCallback(stdr_msgs::LoadExternalMap::Request& req,
        stdr_msgs::LoadExternalMap::Response& res);
      
      //!< Actions  --------------------------
      
      /**
      @brief Action callback for robot spawning
      @param goal [const stdr_msgs::SpawnRobotGoalConstPtr&] The action goal
      @return void
      **/
      void spawnRobotCallback(const stdr_msgs::SpawnRobotGoalConstPtr& goal);
      
      /**
      @brief Action callback for robot deletion
      @param goal [const stdr_msgs::DeleteRobotGoalConstPtr&] The action goal
      @return void
      **/
      void deleteRobotCallback(
        const stdr_msgs::DeleteRobotGoalConstPtr&  goal);
        
      /**
      @brief Action callback for robot registering
      @param goal [const stdr_msgs::RegisterRobotGoalConstPtr&] The action goal
      @return void
      **/
      void registerRobotCallback(
        const stdr_msgs::RegisterRobotGoalConstPtr& goal);
      
    private:
      
      /**
      @brief Initializes the spawn,delete,register action servers
      @return void
      **/
      void activateActionServers(void);
      
      /**
      @brief Adds new robot to simulator
      @param description [stdr_msgs::RobotMsg] The new robot description
      @param result [stdr_msgs::SpawnRobotResult*] The action result
      @return bool
      **/
      bool addNewRobot(stdr_msgs::RobotMsg description, 
        stdr_msgs::SpawnRobotResult* result);
        
      /**
      @brief Deletes a robot from simulator
      @param name [std::string] The robot frame_id
      @param result [stdr_msgs::DeleteRobotResult*] The action result
      @return bool
      **/
      bool deleteRobot(std::string name, stdr_msgs::DeleteRobotResult* result);
      
    private:
    
      //!< THe ROS node handle
      ros::NodeHandle _nh;
      //!< A pointer to a MapServe object
      MapServerPtr _mapServer;
      
      //!< ROS publisher for the ensemble of robots
      ros::Publisher _robotsPublisher;
      
      //!< Action client for robot spawning
      ros::ServiceClient _spawnRobotClient;
      //!< Action client for robot unloading
      ros::ServiceClient _unloadRobotClient;
      //!< Service server for loading maps from files
      ros::ServiceServer _loadMapService;
      //!< Service server for loading maps from GUI
      ros::ServiceServer _loadExternalMapService;
      //!< Service server for moving robots
      ros::ServiceServer _moveRobotService;
      
      //!< Action server for registering robots
      RegisterRobotServer _registerRobotServer;
      //!< Action server for spawning robots
      SpawnRobotServer _spawnRobotServer;
      //!< Action server for deleting robots
      DeleteRobotServer _deleteRobotServer;
      
      //!< An std::map that contains the robots based on theor frame_id
      RobotMap _robotMap;
      //!< Index that shows the next robot id
      int _id;
      
      //!< Boost mutex for conflict avoidance
      boost::mutex _mut;
      //!< Boost condition variable for conflicting avoidance
      boost::condition_variable cond;
  };
}


#endif
