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

#include <stdr_server/stdr_server.h>

namespace stdr_server {

  /**
  @class Server
  @brief Implements the STDR server functionalities
  **/
  Server::Server(int argc, char** argv)
    :_spawnRobotServer(_nh, "stdr_server/spawn_robot",
      boost::bind(&Server::spawnRobotCallback, this, _1), false)

    ,_registerRobotServer(_nh, "stdr_server/register_robot",
      boost::bind(&Server::registerRobotCallback, this, _1), false)

    ,_deleteRobotServer(_nh, "stdr_server/delete_robot",
      boost::bind(&Server::deleteRobotCallback, this, _1), false)

    ,_id(0)
  {
  //~ _spawnRobotServer.registerGoalCallback(
    //~ boost::bind(&Server::spawnRobotCallback, this) );

    if (argc > 2) {
      ROS_ERROR("%s", USAGE);
      exit(-1);
    }

    if (argc == 2) {
      std::string fname(argv[1]);
      _mapServer.reset(new MapServer(fname));

      //!< if we don't have map, no point to start servers
      activateActionServers();
    }

    _loadMapService = _nh.advertiseService(
      "/stdr_server/load_static_map", &Server::loadMapCallback, this);

    _loadExternalMapService = _nh.advertiseService(
      "/stdr_server/load_static_map_external",
        &Server::loadExternalMapCallback, this);

    while (!ros::service::waitForService("robot_manager/load_nodelet",
        ros::Duration(.1)) && ros::ok())
    {
      ROS_WARN("Trying to register to robot_manager/load_nodelet...");
    }

    _spawnRobotClient = _nh.serviceClient<nodelet::NodeletLoad>
      ("robot_manager/load_nodelet", true);

    while (!ros::service::waitForService("robot_manager/unload_nodelet",
      ros::Duration(.1)) && ros::ok())
    {
      ROS_WARN("Trying to register to robot_manager/unload_nodelet...");
    }

    _unloadRobotClient =
      _nh.serviceClient<nodelet::NodeletUnload>("robot_manager/unload_nodelet");

    _robotsPublisher =
      _nh.advertise<stdr_msgs::RobotIndexedVectorMsg>
        ("stdr_server/active_robots", 10, true);

    //!< Rviz publisher
    _sourceVectorPublisherRviz = _nh.advertise<visualization_msgs::MarkerArray>(
      "stdr_server/sources_visualization_markers", 1, true);

    //!< Rfid tags

    _addRfidTagServiceServer = _nh.advertiseService("stdr_server/add_rfid_tag",
      &Server::addRfidTagCallback, this);

    _deleteRfidTagServiceServer =
      _nh.advertiseService("stdr_server/delete_rfid_tag",
      &Server::deleteRfidTagCallback, this);

    _rfidTagVectorPublisher = _nh.advertise<stdr_msgs::RfidTagVector>(
      "stdr_server/rfid_list", 1, true);

    //!< CO2 sources

    _addCO2SourceServiceServer = _nh.advertiseService(
      "stdr_server/add_co2_source",
      &Server::addCO2SourceCallback, this);

    _deleteCO2SourceServiceServer =
      _nh.advertiseService("stdr_server/delete_co2_source",
      &Server::deleteCO2SourceCallback, this);

    _CO2SourceVectorPublisher = _nh.advertise<stdr_msgs::CO2SourceVector>(
      "stdr_server/co2_sources_list", 1, true);

    //!< Thermal sources

    _addThermalSourceServiceServer = _nh.advertiseService(
      "stdr_server/add_thermal_source",
      &Server::addThermalSourceCallback, this);

    _deleteThermalSourceServiceServer =
      _nh.advertiseService("stdr_server/delete_thermal_source",
      &Server::deleteThermalSourceCallback, this);

    _thermalSourceVectorPublisher =
      _nh.advertise<stdr_msgs::ThermalSourceVector>(
      "stdr_server/thermal_sources_list", 1, true);

    //!< Sound sources

    _addSoundSourceServiceServer = _nh.advertiseService(
      "stdr_server/add_sound_source",
      &Server::addSoundSourceCallback, this);

    _deleteSoundSourceServiceServer =
      _nh.advertiseService("stdr_server/delete_sound_source",
      &Server::deleteSoundSourceCallback, this);

    _soundSourceVectorPublisher = _nh.advertise<stdr_msgs::SoundSourceVector>(
      "stdr_server/sound_sources_list", 1, true);
  }

  /**
  @brief Service callback for adding new rfid tag to the environment
  **/
  bool Server::addRfidTagCallback(
    stdr_msgs::AddRfidTag::Request &req,
    stdr_msgs::AddRfidTag::Response &res)
  {
    //!< Sanity check for the RFID tag
    stdr_msgs::RfidTag new_rfid = req.newTag;
    if(_rfidTagMap.find(new_rfid.tag_id) != _rfidTagMap.end())
    { //!< A rfid tag exists with the same tag_id
      res.success = false;
      res.message = "Duplicate rfid_id";
      return false;
    }

    //!< Add RFID tag to the environment
    _rfidTagMap.insert(std::pair<std::string, stdr_msgs::RfidTag>(
      new_rfid.tag_id, new_rfid));

    //!< Publish the new RFID tag list
    stdr_msgs::RfidTagVector rfidTagList;
    visualization_msgs::MarkerArray RFIDMarkerArray;

    for(RfidTagMapIt it = _rfidTagMap.begin() ; it != _rfidTagMap.end() ; it++)
    {
      rfidTagList.rfid_tags.push_back(it->second);
      RFIDMarkerArray.markers.push_back(toMarker(it->second, true));
    }
    _rfidTagVectorPublisher.publish(rfidTagList);

    _sourceVectorPublisherRviz.publish(RFIDMarkerArray);

    //!< Republish existing sources as markers
    republishSources();

    //!< Return success
    res.success = true;
    return true;
  }

  /**
  @brief Service callback for adding new CO2 source to the environment
  **/
  bool Server::addCO2SourceCallback(
    stdr_msgs::AddCO2Source::Request &req,
    stdr_msgs::AddCO2Source::Response &res)
  {
    //!< Sanity check for the source
    stdr_msgs::CO2Source new_source = req.newSource;
    if(_CO2SourceMap.find(new_source.id) != _CO2SourceMap.end())
    { //!< A source exists with the same id
      res.success = false;
      res.message = "Duplicate CO2 id";
      return false;
    }

    //!< Add CO2 source to the environment
    _CO2SourceMap.insert(std::pair<std::string, stdr_msgs::CO2Source>(
      new_source.id, new_source));

    //!< Publish the new source list to both our custom GUI and Rviz.
    stdr_msgs::CO2SourceVector CO2SourceList;
    visualization_msgs::MarkerArray C02MarkerArray;

    for(CO2SourceMapIt it = _CO2SourceMap.begin()
      ; it != _CO2SourceMap.end() ; it++)
    {
      CO2SourceList.co2_sources.push_back(it->second);
      C02MarkerArray.markers.push_back(toMarker(it->second, true));
    }
    _CO2SourceVectorPublisher.publish(CO2SourceList);
    _sourceVectorPublisherRviz.publish(C02MarkerArray);


    //!< Republish existing sources as markers
    republishSources();

    //!< Return success
    res.success = true;
    return true;
  }

  /**
  @brief Service callback for adding new thermal source to the environment
  **/
  bool Server::addThermalSourceCallback(
    stdr_msgs::AddThermalSource::Request &req,
    stdr_msgs::AddThermalSource::Response &res)
  {
    //!< Sanity check for the source
    stdr_msgs::ThermalSource new_source = req.newSource;
    if(_thermalSourceMap.find(new_source.id) != _thermalSourceMap.end())
    { //!< A source exists with the same id
      res.success = false;
      res.message = "Duplicate thermal source is";
      return false;
    }

    //!< Add source to the environment
    _thermalSourceMap.insert(std::pair<std::string, stdr_msgs::ThermalSource>(
      new_source.id, new_source));

    //!< Publish the new source list
    stdr_msgs::ThermalSourceVector thermalSourceList;
    visualization_msgs::MarkerArray thermalMarkerArray;

    for(ThermalSourceMapIt it = _thermalSourceMap.begin()
      ; it != _thermalSourceMap.end() ; it++)
    {
      thermalSourceList.thermal_sources.push_back(it->second);
      thermalMarkerArray.markers.push_back(toMarker(it->second, true));
    }
    _thermalSourceVectorPublisher.publish(thermalSourceList);
    _sourceVectorPublisherRviz.publish(thermalMarkerArray);


    //!< Republish existing sources as markers
    republishSources();

    //!< Return success
    res.success = true;
    return true;
  }

  /**
  @brief Service callback for adding new sound source to the environment
  **/
  bool Server::addSoundSourceCallback(
    stdr_msgs::AddSoundSource::Request &req,
    stdr_msgs::AddSoundSource::Response &res)
  {
    //!< Sanity check for the source
    stdr_msgs::SoundSource new_source = req.newSource;
    if(_soundSourceMap.find(new_source.id) != _soundSourceMap.end())
    { //!< A source exists with the same id
      res.success = false;
      res.message = "Duplicate sound source is";
      return false;
    }

    //!< Add source to the environment
    _soundSourceMap.insert(std::pair<std::string, stdr_msgs::SoundSource>(
      new_source.id, new_source));

    //!< Publish the new source list
    stdr_msgs::SoundSourceVector soundSourceList;
    visualization_msgs::MarkerArray soundMarkerArray;

    for(SoundSourceMapIt it = _soundSourceMap.begin()
      ; it != _soundSourceMap.end() ; it++)
    {
      soundSourceList.sound_sources.push_back(it->second);
      soundMarkerArray.markers.push_back(toMarker(it->second, true));
    }
    _soundSourceVectorPublisher.publish(soundSourceList);
    _sourceVectorPublisherRviz.publish(soundMarkerArray);


    //!< Republish existing sources as markers
    republishSources();

    //!< Return success
    res.success = true;
    return true;
  }

  /**
  @brief Service callback for deleting an rfid tag from the environment
  **/
  bool Server::deleteRfidTagCallback(
    stdr_msgs::DeleteRfidTag::Request &req,
    stdr_msgs::DeleteRfidTag::Response &res)
  {

    std::string name = req.name;
    if(_rfidTagMap.find(name) != _rfidTagMap.end())
    {
      // Publish deletion to RVIZ
      visualization_msgs::MarkerArray rfidMarkerArray;
      rfidMarkerArray.markers.push_back(toMarker(_rfidTagMap[name], false));
      _sourceVectorPublisherRviz.publish(rfidMarkerArray);

      _rfidTagMap.erase(name);

      //!< Publish the new RFID tag list
      stdr_msgs::RfidTagVector rfidTagList;

      for(RfidTagMapIt it = _rfidTagMap.begin() ;
        it != _rfidTagMap.end() ; it++)
      {
        rfidTagList.rfid_tags.push_back(it->second);
      }
      _rfidTagVectorPublisher.publish(rfidTagList);
      //!< Republish existing sources to RVIZ.
      republishSources();
    }
    else  //!< Tag does not exist
    {
      return false;
    }
    return true;
  }

  /**
  @brief Service callback for deleting a CO2 source from the environment
  **/
  bool Server::deleteCO2SourceCallback(
    stdr_msgs::DeleteCO2Source::Request &req,
    stdr_msgs::DeleteCO2Source::Response &res)
  {
    std::string name = req.name;
    if(_CO2SourceMap.find(name) != _CO2SourceMap.end())
    {
      // Publish deletion to RVIZ
      visualization_msgs::MarkerArray CO2MarkerArray;
      CO2MarkerArray.markers.push_back(toMarker(_CO2SourceMap[name], false));
      _sourceVectorPublisherRviz.publish(CO2MarkerArray);

      _CO2SourceMap.erase(name);

      //!< Publish the new sources list
      stdr_msgs::CO2SourceVector CO2SourceList;

      for(CO2SourceMapIt it = _CO2SourceMap.begin() ;
        it != _CO2SourceMap.end() ; it++)
      {
        CO2SourceList.co2_sources.push_back(it->second);
      }
      _CO2SourceVectorPublisher.publish(CO2SourceList);
      //!< Republish existing sources to RVIZ.
      republishSources();
    }

    else  //!< Source does not exist
    {
      return false;
    }
    return true;
  }

  /**
  @brief Service callback for deleting a thermal source from the environment
  **/
  bool Server::deleteThermalSourceCallback(
    stdr_msgs::DeleteThermalSource::Request &req,
    stdr_msgs::DeleteThermalSource::Response &res)
  {

    std::string name = req.name;
    if(_thermalSourceMap.find(name) != _thermalSourceMap.end())
    {
      // Publish deletion to RVIZ
      visualization_msgs::MarkerArray thermalMarkerArray;
      thermalMarkerArray.markers.push_back(toMarker(_thermalSourceMap[name], false));
      _sourceVectorPublisherRviz.publish(thermalMarkerArray);

      _thermalSourceMap.erase(name);

      //!< Publish the new sources list
      stdr_msgs::ThermalSourceVector thermalSourceList;

      for(ThermalSourceMapIt it = _thermalSourceMap.begin() ;
        it != _thermalSourceMap.end() ; it++)
      {
        thermalSourceList.thermal_sources.push_back(it->second);
      }
      _thermalSourceVectorPublisher.publish(thermalSourceList);
      //!< Republish existing sources to RVIZ.
      republishSources();
    }
    else  //!< Source does not exist
    {
      return false;
    }
    return true;
  }

  /**
  @brief Service callback for deleting a sound source from the environment
  **/
  bool Server::deleteSoundSourceCallback(
    stdr_msgs::DeleteSoundSource::Request &req,
    stdr_msgs::DeleteSoundSource::Response &res)
  {

    std::string name = req.name;
    if(_soundSourceMap.find(name) != _soundSourceMap.end())
    {
      // Publish deletion to RVIZ
      visualization_msgs::MarkerArray soundMarkerArray;
      soundMarkerArray.markers.push_back(toMarker(_soundSourceMap[name], false));
      _sourceVectorPublisherRviz.publish(soundMarkerArray);

      _soundSourceMap.erase(name);

      //!< Publish the new sources list
      stdr_msgs::SoundSourceVector soundSourceList;


      for(SoundSourceMapIt it = _soundSourceMap.begin() ;
        it != _soundSourceMap.end() ; it++)
      {
        soundSourceList.sound_sources.push_back(it->second);

      }
      _soundSourceVectorPublisher.publish(soundSourceList);
      //!< Republish existing sources to RVIZ.
      republishSources();
    }
    else  //!< Source does not exist
    {
      return false;
    }
    return true;
  }

  /**
  @brief Service callback for loading the map
  @param req [stdr_msgs::LoadMap::Request&] The service request
  @param res [stdr_msgs::LoadMap::Response&] The service response
  @return bool
  **/
  bool Server::loadMapCallback(
    stdr_msgs::LoadMap::Request& req,
    stdr_msgs::LoadMap::Response& res)
  {
    if (_mapServer) {
      ROS_WARN("Map already loaded!");
      return false;
    }
    _mapServer.reset(new MapServer(req.mapFile));
    //!< if we don't have map, no point to start servers
    activateActionServers();

    return true;
  }

  /**
  @brief Service callback for loading the map from GUI
  @param req [stdr_msgs::LoadExternalMap::Request&] The service request
  @param res [stdr_msgs::LoadExternalMap::Response&] The service response
  @return bool
  **/
  bool Server::loadExternalMapCallback(
    stdr_msgs::LoadExternalMap::Request& req,
    stdr_msgs::LoadExternalMap::Response& res)
  {
    if (_mapServer) {
      ROS_WARN("Map already loaded!");
      return false;
    }
    _mapServer.reset(new MapServer(req.map));

    //!< if we don't have map, no point to start servers
    activateActionServers();

    return true;
  }

  /**
  @brief Action callback for robot spawning
  @param goal [const stdr_msgs::SpawnRobotGoalConstPtr&] The action goal
  @return void
  **/
  void Server::spawnRobotCallback(
    const stdr_msgs::SpawnRobotGoalConstPtr& goal)
  {
    stdr_msgs::SpawnRobotResult result;

    std::string f_id;
    if(hasDublicateFrameIds(goal->description, f_id))
    {
      result.message = std::string("Double frame_id:") + f_id;
      _spawnRobotServer.setAborted(result);
      return;
    }

    if (addNewRobot(goal->description, &result)) {
      _spawnRobotServer.setSucceeded(result);

      //!< publish to active_robots topic
      stdr_msgs::RobotIndexedVectorMsg msg;
      for (RobotMap::iterator it = _robotMap.begin();
        it != _robotMap.end(); ++it)
      {
        msg.robots.push_back( it->second );
      }

      _robotsPublisher.publish(msg);
      return;
    }

    _spawnRobotServer.setAborted(result);
  }

  /**
  @brief Action callback for robot deletion
  @param goal [const stdr_msgs::DeleteRobotGoalConstPtr&] The action goal
  @return void
  **/
  void Server::deleteRobotCallback(
    const stdr_msgs::DeleteRobotGoalConstPtr&  goal)
  {

    stdr_msgs::DeleteRobotResult result;

    if (deleteRobot(goal->name, &result)) {

      // publish to active_robots topic
      stdr_msgs::RobotIndexedVectorMsg msg;
      for (RobotMap::iterator it = _robotMap.begin();
        it != _robotMap.end(); ++it)
      {
        msg.robots.push_back( it->second );
      }
      _robotsPublisher.publish(msg);
      _deleteRobotServer.setSucceeded(result);
      return;
    }

    _deleteRobotServer.setAborted(result);
  }

  /**
  @brief Action callback for robot registering
  @param goal [const stdr_msgs::RegisterRobotGoalConstPtr&] The action goal
  @return void
  **/
  void Server::registerRobotCallback(
    const stdr_msgs::RegisterRobotGoalConstPtr& goal)
  {

    boost::unique_lock<boost::mutex> lock(_mut);
    stdr_msgs::RegisterRobotResult result;

    // look for the robot name in _robotMap
    RobotMap::iterator search = _robotMap.find(goal->name);

    if (search != _robotMap.end())
    {
      result.description = _robotMap[goal->name].robot;
      _registerRobotServer.setSucceeded(result);

      //!< notify spawn action, to reply to spawnner
      _cond.notify_one();
    }
    else
    {
      _registerRobotServer.setAborted(result);
    }
  }

  /**
  @brief Initializes the spawn,delete,register action servers
  @return void
  **/
  void Server::activateActionServers(void)
  {
    _spawnRobotServer.start();
    _registerRobotServer.start();
    _deleteRobotServer.start();
  }

  /**
  @brief Adds new robot to simulator
  @param description [stdr_msgs::RobotMsg] The new robot description
  @param result [stdr_msgs::SpawnRobotResult*] The action result
  @return bool
  **/
  bool Server::addNewRobot(
    stdr_msgs::RobotMsg description,
    stdr_msgs::SpawnRobotResult* result)
  {

    stdr_msgs::RobotIndexedMsg namedRobot;

    if(description.kinematicModel.type == "")
      description.kinematicModel.type = "ideal";

    namedRobot.robot = description;

    namedRobot.name = "robot" + boost::lexical_cast<std::string>(_id++);

    _robotMap.insert( std::make_pair(namedRobot.name, namedRobot) );

    nodelet::NodeletLoad srv;
    srv.request.name = namedRobot.name;
    srv.request.type = "stdr_robot/Robot";

    boost::unique_lock<boost::mutex> lock(_mut);

    if (_spawnRobotClient.call(srv)) {
      //!< wait until robot calls RobotRegisterAction or timeout expires
      if( _cond.timed_wait(lock, ros::Duration(5.0).toBoost()) )
      {
        result->indexedDescription = namedRobot;

        lock.unlock();
        return true;
      }
    }

    result->message =
      "Robot didn't respond within time or registered with incorrect name.";
    lock.unlock();
    return false;
  }

  /**
  @brief Deletes a robot from simulator
  @param name [std::string] The robot frame_id
  @param result [stdr_msgs::DeleteRobotResult*] The action result
  @return bool
  **/
  bool Server::deleteRobot(
    std::string name,
    stdr_msgs::DeleteRobotResult* result)
  {

    RobotMap::iterator it = _robotMap.find(name);

    if (it != _robotMap.end()) {

      nodelet::NodeletUnload srv;
      srv.request.name =  name;

      if (_unloadRobotClient.call(srv)) {

        if (srv.response.success) {
          _robotMap.erase(it);
        }

        result->success = srv.response.success;
        return srv.response.success;
      }

      result->success = false;
      return false;
    }

    ROS_WARN("Requested to delete robot, with name %s does not exist.",
      name.c_str());

    result->success = false;
    return false;
  }

  bool Server::hasDublicateFrameIds(const stdr_msgs::RobotMsg& robot,
    std::string &f_id)
  {
    std::set<std::string> f_ids;
    for(unsigned int i = 0 ; i < robot.laserSensors.size() ; i++)
    {
      if(f_ids.find(robot.laserSensors[i].frame_id) == f_ids.end())
      {
        f_ids.insert(robot.laserSensors[i].frame_id);
      }
      else
      {
        f_id = robot.laserSensors[i].frame_id;
        return true;
      }
    }
    for(unsigned int i = 0 ; i < robot.sonarSensors.size() ; i++)
    {
      if(f_ids.find(robot.sonarSensors[i].frame_id) == f_ids.end())
      {
        f_ids.insert(robot.sonarSensors[i].frame_id);
      }
      else
      {
        f_id = robot.sonarSensors[i].frame_id;
        return true;
      }
    }
    for(unsigned int i = 0 ; i < robot.rfidSensors.size() ; i++)
    {
      if(f_ids.find(robot.rfidSensors[i].frame_id) == f_ids.end())
      {
        f_ids.insert(robot.rfidSensors[i].frame_id);
      }
      else
      {
        f_id = robot.rfidSensors[i].frame_id;
        return true;
      }
    }
    for(unsigned int i = 0 ; i < robot.co2Sensors.size() ; i++)
    {
      if(f_ids.find(robot.co2Sensors[i].frame_id) == f_ids.end())
      {
        f_ids.insert(robot.co2Sensors[i].frame_id);
      }
      else
      {
        f_id = robot.co2Sensors[i].frame_id;
        return true;
      }
    }
    for(unsigned int i = 0 ; i < robot.soundSensors.size() ; i++)
    {
      if(f_ids.find(robot.soundSensors[i].frame_id) == f_ids.end())
      {
        f_ids.insert(robot.soundSensors[i].frame_id);
      }
      else
      {
        f_id = robot.soundSensors[i].frame_id;
        return true;
      }
    }
    for(unsigned int i = 0 ; i < robot.thermalSensors.size() ; i++)
    {
      if(f_ids.find(robot.thermalSensors[i].frame_id) == f_ids.end())
      {
        f_ids.insert(robot.thermalSensors[i].frame_id);
      }
      else
      {
        f_id = robot.thermalSensors[i].frame_id;
        return true;
      }
    }
    return false;
  }

  /**
  @brief Translate the stdr_C02Source message into a marker message
  **/
  visualization_msgs::Marker Server::toMarker(const stdr_msgs::CO2Source& msg, bool added)
  {
    visualization_msgs::Marker marker = createMarker(msg, added);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "co2";
    marker.id = atoi(msg.id.c_str());

    // Set the color specific to the source type -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
  }

  /**
  @brief Translate the stdr_ThermalSource message into a marker message
  **/
  visualization_msgs::Marker Server::toMarker(const stdr_msgs::ThermalSource& msg, bool added)
  {
    visualization_msgs::Marker marker = createMarker(msg, added);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "thermal";
    marker.id = atoi(msg.id.c_str());

    // Set the color specific to the source type -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
  }

  /**
  @brief Translate the stdr_SoundSource message into a marker message
  **/
  visualization_msgs::Marker Server::toMarker(const stdr_msgs::SoundSource& msg, bool added)
  {
    visualization_msgs::Marker marker = createMarker(msg, added);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "sound";
    marker.id = atoi(msg.id.c_str());

    // Set the color specific to the source type -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    return marker;
  }

  /**
  @brief Translate the stdr_SoundSource message into a marker message
  **/
  visualization_msgs::Marker Server::toMarker(const stdr_msgs::RfidTag& msg, bool added)
  {
    visualization_msgs::Marker marker = createMarker(msg, added);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "rfid";
    marker.id = atoi(msg.tag_id.c_str());

    // Set the color specific to the source type -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
  }

  void Server::republishSources()
  {
    visualization_msgs::MarkerArray ma;
    for(SoundSourceMapIt it = _soundSourceMap.begin()
      ; it != _soundSourceMap.end() ; it++)
    {
        ma.markers.push_back(toMarker(it->second, true));
    }
    for(CO2SourceMapIt it = _CO2SourceMap.begin()
      ; it != _CO2SourceMap.end() ; it++)
    {
      ma.markers.push_back(toMarker(it->second, true));
    }
    for(ThermalSourceMapIt it = _thermalSourceMap.begin()
      ; it != _thermalSourceMap.end() ; it++)
    {
      ma.markers.push_back(toMarker(it->second, true));
    }
    for(RfidTagMapIt it = _rfidTagMap.begin() ; it != _rfidTagMap.end() ; it++)
    {
      ma.markers.push_back(toMarker(it->second, true));
    }
    _sourceVectorPublisherRviz.publish(ma);
  }

  /**
  @brief Creates a marker message corresponding to every element of msg that is
  independent of the source's specific type
  **/
  template <class SourceMsg>
  visualization_msgs::Marker Server::createMarker(const SourceMsg& msg, bool added)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map_static";
    marker.header.stamp = ros::Time();

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker.type = shape;

    // Set the marker action.
    if(added) {
      marker.action = visualization_msgs::Marker::ADD;
    }
    else {
      marker.action = visualization_msgs::Marker::DELETE;
    }

    marker.pose.position.x = msg.pose.x;
    marker.pose.position.y = msg.pose.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.lifetime = ros::Duration();
    return marker;
  }

} // end of namespace stdr_robot
