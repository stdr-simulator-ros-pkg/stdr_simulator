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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <stdr_msgs/RfidTagVector.h>
#include <stdr_msgs/AddRfidTag.h>
#include <stdr_msgs/DeleteRfidTag.h>

#include <stdr_msgs/CO2SourceVector.h>
#include <stdr_msgs/AddCO2Source.h>
#include <stdr_msgs/DeleteCO2Source.h>

#include <stdr_msgs/ThermalSourceVector.h>
#include <stdr_msgs/AddThermalSource.h>
#include <stdr_msgs/DeleteThermalSource.h>

#include <stdr_msgs/SoundSourceVector.h>
#include <stdr_msgs/AddSoundSource.h>
#include <stdr_msgs/DeleteSoundSource.h>

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

  typedef std::map<std::string, stdr_msgs::RfidTag> RfidTagMap;
  typedef std::map<std::string, stdr_msgs::RfidTag>::iterator RfidTagMapIt;

  typedef std::map<std::string, stdr_msgs::CO2Source> CO2SourceMap;
  typedef std::map<std::string, stdr_msgs::CO2Source>::iterator CO2SourceMapIt;

  typedef std::map<std::string, stdr_msgs::ThermalSource> ThermalSourceMap;
  typedef std::map<std::string, stdr_msgs::ThermalSource>::iterator
    ThermalSourceMapIt;

  typedef std::map<std::string, stdr_msgs::SoundSource> SoundSourceMap;
  typedef std::map<std::string, stdr_msgs::SoundSource>::iterator
    SoundSourceMapIt;

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

      /**
      @brief Service callback for adding new rfid tag to the environment
      @param req [stdr_msgs::AddRfidTag::Request &] The request
      @param res [stdr_msgs::AddRfidTag::Response &] The Response
      @return bool
      **/
      bool addRfidTagCallback(
        stdr_msgs::AddRfidTag::Request &req,
        stdr_msgs::AddRfidTag::Response &res);

      /**
      @brief Service callback for deleting an rfid tag from the environment
      @param req [stdr_msgs::DeleteRfidTag::Request &] The request
      @param res [stdr_msgs::DeleteRfidTag::Response &] The Response
      @return bool
      **/
      bool deleteRfidTagCallback(
        stdr_msgs::DeleteRfidTag::Request &req,
        stdr_msgs::DeleteRfidTag::Response &res);

      /**
      @brief Service callback for adding new CO2 source to the environment
      @param req [stdr_msgs::AddCO2Source::Request &] The request
      @param res [stdr_msgs::AddCO2Source::Response &] The Response
      @return bool
      **/
      bool addCO2SourceCallback(
        stdr_msgs::AddCO2Source::Request &req,
        stdr_msgs::AddCO2Source::Response &res);

      /**
      @brief Service callback for deleting a CO2 source from the environment
      @param req [stdr_msgs::DeleteCO2Source::Request &] The request
      @param res [stdr_msgs::DeleteCO2Source::Response &] The Response
      @return bool
      **/
      bool deleteCO2SourceCallback(
        stdr_msgs::DeleteCO2Source::Request &req,
        stdr_msgs::DeleteCO2Source::Response &res);

      /**
      @brief Service callback for adding new thermal source to the environment
      @param req [stdr_msgs::AddThermalSource::Request &] The request
      @param res [stdr_msgs::AddThermalSource::Response &] The Response
      @return bool
      **/
      bool addThermalSourceCallback(
        stdr_msgs::AddThermalSource::Request &req,
        stdr_msgs::AddThermalSource::Response &res);

      /**
      @brief Service callback for deleting a thermal source from the environment
      @param req [stdr_msgs::DeleteThermalSource::Request &] The request
      @param res [stdr_msgs::DeleteThermalSource::Response &] The Response
      @return bool
      **/
      bool deleteThermalSourceCallback(
        stdr_msgs::DeleteThermalSource::Request &req,
        stdr_msgs::DeleteThermalSource::Response &res);

      /**
      @brief Service callback for adding new sound source to the environment
      @param req [stdr_msgs::AddSoundSource::Request &] The request
      @param res [stdr_msgs::AddSoundSource::Response &] The Response
      @return bool
      **/
      bool addSoundSourceCallback(
        stdr_msgs::AddSoundSource::Request &req,
        stdr_msgs::AddSoundSource::Response &res);

      /**
      @brief Service callback for deleting a sound source from the environment
      @param req [stdr_msgs::DeleteSoundSource::Request &] The request
      @param res [stdr_msgs::DeleteSoundSource::Response &] The Response
      @return bool
      **/
      bool deleteSoundSourceCallback(
        stdr_msgs::DeleteSoundSource::Request &req,
        stdr_msgs::DeleteSoundSource::Response &res);

      bool hasDublicateFrameIds(const stdr_msgs::RobotMsg& robot,
        std::string &f_id);

      /**
      @brief Translate the stdr_C02Source message into a marker message
      @param msg [stdr_msgs::CO2Source] The GUI message to be translated
      @param added [bool] True if the marker is added, false if it should be deleted
      @return [visualization_msgs::Marker] the marker message to be published to Rviz
      **/
      visualization_msgs::Marker toMarker(const stdr_msgs::CO2Source& msg,bool added);

      /**
      @brief Translate the stdr_ThermalSource message into a marker message
      @param msg [stdr_msgs::ThermalSource] The GUI message to be translated
      @param added [bool] True if the marker is added, false if it should be deleted
      @return [visualization_msgs::Marker] the marker message to be published to Rviz
      **/
      visualization_msgs::Marker toMarker(const stdr_msgs::ThermalSource& msg,bool added);

      /**
      @brief Translate the stdr_SoundSource message into a marker message
      @param msg [stdr_msgs::SoundSource] The GUI message to be translated
      @param added [bool] True if the marker is added, false if it should be deleted
      @return [visualization_msgs::Marker] the marker message to be published to Rviz
      **/
      visualization_msgs::Marker toMarker(const stdr_msgs::SoundSource& msg,bool added);

      /**
      @brief Translate the stdr_RfidTag message into a marker message
      @param msg [stdr_msgs::RfidTag] The GUI message to be translated
      @param added [bool] True if the marker is added, false if it should be deleted
      @return [visualization_msgs::Marker] the marker message to be published to Rviz
      **/
      visualization_msgs::Marker toMarker(const stdr_msgs::RfidTag& msg,bool added);

      /**
      @brief Republishes existing sources to RVIZ after a successful deletion.
      @return [void]
      **/
      void republishSources();

      /**
      @brief Creates a marker message corresponding to every element of msg that is
      independent of the source's specific type
      @param msg [SourceMsg] The source message
      @param added [bool] True if the marker is added, false if it should be deleted
      @return [visualization_msgs::Marker] the marker message to be published to Rviz
      **/
      template <class SourceMsg>
      visualization_msgs::Marker createMarker(const SourceMsg& msg,bool added);

      //!< The ROS node handle
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

      //!< An std::map that contains the rfid tags existent in the environment
      RfidTagMap _rfidTagMap;
      //!< An std::map that contains the CO2 sources existent in the environment
      CO2SourceMap _CO2SourceMap;
      //!< An std::map that contains the thermal sources existent in the environment
      ThermalSourceMap _thermalSourceMap;
      //!< An std::map that contains the sound sources existent in the environment
      SoundSourceMap _soundSourceMap;

      //!< Boost mutex for conflict avoidance
      boost::mutex _mut;
      //!< Boost condition variable for conflicting avoidance
      boost::condition_variable _cond;


      //!< A general Rviz publisher for all source types
      ros::Publisher _sourceVectorPublisherRviz;

      //!< The addRfidTag srv server
      ros::ServiceServer _addRfidTagServiceServer;
      //!< The deleteRfidTag srv server
      ros::ServiceServer _deleteRfidTagServiceServer;
      //!< The rfid tag list publisher
      ros::Publisher _rfidTagVectorPublisher;


      //!< The addCO2Source srv server
      ros::ServiceServer _addCO2SourceServiceServer;
      //!< The deleteCO2Source srv server
      ros::ServiceServer _deleteCO2SourceServiceServer;
      //!< The CO2 source list publisher towards the GUI.
      ros::Publisher _CO2SourceVectorPublisher;

      //!< The addThermalSource srv server
      ros::ServiceServer _addThermalSourceServiceServer;
      //!< The deleteThermalSource srv server
      ros::ServiceServer _deleteThermalSourceServiceServer;
      //!< The thermal source list publisher
      ros::Publisher _thermalSourceVectorPublisher;

      //!< The addSoundSource srv server
      ros::ServiceServer _addSoundSourceServiceServer;
      //!< The deleteSoundSource srv server
      ros::ServiceServer _deleteSoundSourceServiceServer;
      //!< The sound source list publisher
      ros::Publisher _soundSourceVectorPublisher;
  };
}


#endif
