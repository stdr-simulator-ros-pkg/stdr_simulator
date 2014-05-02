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

#ifndef STDR_GUI_CONTROLLER
#define STDR_GUI_CONTROLLER

#include <iostream>
#include <cstdlib>
#include <boost/thread.hpp>

#include "nav_msgs/OccupancyGrid.h"

#include "stdr_gui/stdr_gui_connector.h"
#include "stdr_gui/stdr_info_connector.h"
#include "stdr_gui/stdr_map_connector.h"
#include "stdr_gui/stdr_visualization/stdr_sonar_visualization.h"
#include "stdr_gui/stdr_visualization/stdr_laser_visualization.h"
#include "stdr_gui/stdr_visualization/stdr_robot_visualization.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_co2_source.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_sound_source.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_thermal_source.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_rfid_tag.h"

#include <stdr_robot/handle_robot.h>

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{

  /**
  @class GuiController
  @brief The main controller for the STDR GUI. Inherits QThread
  **/ 
  class CGuiController : 
    public QThread
  {
    Q_OBJECT
    
    //------------------------------------------------------------------------//
    private: 

      typedef std::map<QString,CLaserVisualisation *>::iterator
        LaserVisIterator;
      typedef std::map<QString,CSonarVisualisation *>::iterator
        SonarVisIterator;
      typedef std::map<QString,CRobotVisualisation *>::iterator
        RobotVisIterator;
      typedef std::map<QString,CGuiRfidTag>::iterator RfidTagIterator;
      typedef std::map<QString,CGuiCo2Source>::iterator Co2SourcesIterator;
      typedef std::map<QString,CGuiSoundSource>::iterator SoundSourcesIterator;
      typedef std::map<QString,CGuiThermalSource>::iterator 
        ThermalSourcesIterator;
      
      //!< Number of input arguments
      int  argc_;
      //!< Input arguments
      char** argv_;
      
      //!< Prevents concurrent map writing
      bool map_lock_;         
      //!< Prevents actions before map initializes
      bool map_initialized_;
      
      //!< All the robots server has
      std::vector<CGuiRobot> registered_robots_;
      //!< Robots created from the specific GUI instance
      std::set<std::string> my_robots_;
      
      //!< Laser visualizers
      std::map<QString,CLaserVisualisation *> laser_visualizers_;
      //!< Sonar visualizers
      std::map<QString,CSonarVisualisation *> sonar_visualizers_;
      //!< Robot visualizers
      std::map<QString,CRobotVisualisation *> robot_visualizers_;

      //!< RFID tags in the environment
      std::map<QString,CGuiRfidTag> rfid_tags_;
      //!< The original rfid vector
      stdr_msgs::RfidTagVector rfid_tag_pure_;
      //!< Thermal sources in the environment
      stdr_msgs::ThermalSourceVector thermal_source_pure_;
      //!< CO2 sources in the environment
      std::map<QString,CGuiThermalSource> thermal_sources_;
      //!< The original vector
      stdr_msgs::CO2SourceVector co2_source_pure_;
      //!< CO2 sources in the environment
      std::map<QString,CGuiCo2Source> co2_sources_;
      //!< Sound sources in the environment
      stdr_msgs::SoundSourceVector sound_source_pure_;
      //!< Sound sources in the environment
      std::map<QString,CGuiSoundSource> sound_sources_;
      
      //!< ROS subscriber for occupancy grid map
      ros::Subscriber map_subscriber_;
      //!< ROS subscriber to get all robots
      ros::Subscriber robot_subscriber_;
      //!< ROS subscriber for rfids
      ros::Subscriber rfids_subscriber_;
      //!< ROS subscriber for co2 sources
      ros::Subscriber co2_sources_subscriber_;
      //!< ROS subscriber for thermal sources
      ros::Subscriber thermal_sources_subscriber_;
      //!< ROS subscriber for sound sources
      ros::Subscriber sound_sources_subscriber_;
      //!< The ROS node handle
      ros::NodeHandle n_;
      //!< ROS tf transform listener
      tf::TransformListener listener_;
      
      //!< The occypancy grid map
      nav_msgs::OccupancyGrid map_msg_;

      //!< Robot handler from stdr_robot
      stdr_robot::HandleRobot robot_handler_;
      //!< All robots in "raw" form
      stdr_msgs::RobotIndexedVectorMsg all_robots_;

      //!< Timer for the drawing event
      QTimer* timer_;
      //!< Elapsed time from the experiment's start
      QTime elapsed_time_;
      
      //!< QIcon for move robot (pop-up menu)
      QIcon icon_move_;
      //!< QIcon for delete robot (pop-up menu)
      QIcon icon_delete_;
      
      //!< QImage created one time, containing the OGM
      QImage initial_map_;
      //!< QImage that initiates as initial_map and the elements are painted on it
      QImage running_map_;
      
      //!< Object of CGuiConnector
      CGuiConnector gui_connector_;
      //!< Object of CInfoConnector
      CInfoConnector info_connector_;
      //!< Object of CMapConnector
      CMapConnector map_connector_;

      /**
      @brief Returns a stdr_msgs::LaserSensorMsg message from robot and laser name
      @param robotName [QString] Frame id of the robot
      @param laserName [QString] Frame id of the laser
      @return stdr_msgs::LaserSensorMsg
      **/
      stdr_msgs::LaserSensorMsg getLaserDescription(
        QString robotName,
        QString laserName); 
        
      /**
      @brief Returns a stdr_msgs::SonarSensorMsg message from robot and sonar name
      @param robotName [QString] Frame id of the robot
      @param sonarName [QString] Frame id of the sonar
      @return stdr_msgs::SonarSensorMsg
      **/
      stdr_msgs::SonarSensorMsg getSonarDescription(
        QString robotName,
        QString sonarName); 
        
      //!< Frame id of the following robot
      std::string robot_following_;
      
      //!< Service client for inserting a new rfid tag
      ros::ServiceClient new_rfid_tag_client_;
      //!< Service client for deleting a rfid tag
      ros::ServiceClient delete_rfid_tag_client_;
      
      //!< Service client for inserting a new co2 source
      ros::ServiceClient new_co2_source_client_;
      //!< Service client for deleting a co2 source
      ros::ServiceClient delete_co2_source_client_;
      
      //!< Service client for inserting a new thermal source
      ros::ServiceClient new_thermal_source_client_;
      //!< Service client for deleting a thermal source
      ros::ServiceClient delete_thermal_source_client_;
      
      //!< Service client for inserting a new sound source
      ros::ServiceClient new_sound_source_client_;
      //!< Service client for deleting a sound source
      ros::ServiceClient delete_sound_source_client_;
    
    //------------------------------------------------------------------------//  
    public:
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CGuiController(int argc,char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiController(void);
      
      /**
      @brief Sets up the main window widgets
      @return void
      **/
      void setupWidgets(void);
      
      /**
      @brief Initializes the Qt event connections and ROS subscribers and publishers
      @return void
      **/
      void initializeCommunications(void);
      
      /**
      @brief Receives the occupancy grid map from stdr_server. Connects to "map" ROS topic
      @param msg [const nav_msgs::OccupancyGrid&] The OGM message
      @return void
      **/
      void receiveMap(const nav_msgs::OccupancyGrid& msg);
      
      /**
      @brief Receives the existent rfid tags
      @param msg [const stdr_msgs::RfidTagVector&] The rfid tags message
      @return void
      **/
      void receiveRfids(const stdr_msgs::RfidTagVector& msg);
      
      /**
      @brief Receives the existent co2 sources
      @param msg [const stdr_msgs::CO2SourceVector&] The CO2 source message
      @return void
      **/
      void receiveCO2Sources(const stdr_msgs::CO2SourceVector& msg);
      
      /**
      @brief Receives the existent thermal sources
      @param msg [const stdr_msgs::ThermalSourceVector&] The thermal source message
      @return void
      **/
      void receiveThermalSources(const stdr_msgs::ThermalSourceVector& msg);
      
      /**
      @brief Receives the existent sound sources
      @param msg [const stdr_msgs::SoundSourceVector&] The sound source message
      @return void
      **/
      void receiveSoundSources(const stdr_msgs::SoundSourceVector& msg);
      
      /**
      @brief Receives the robots from stdr_server. Connects to "stdr_server/active_robots" ROS topic
      @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The robots message
      @return void
      **/
      void receiveRobots(const stdr_msgs::RobotIndexedVectorMsg& msg);
      
      /**
      @brief Initializes the ROS spin and Qt threads
      @return bool
      **/
      bool init();
      
      /**
      @brief Dumps all visualizers that connect to robots not existent in the input argument message
      @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The robots message
      @return void
      **/
      void cleanupVisualizers(const stdr_msgs::RobotIndexedVectorMsg& msg);
    
    //------------------------------------------------------------------------//
    public Q_SLOTS:
    
      /**
      @brief Saves the robot in a yaml file. Connects to the CGuiConnector::CRobotCreatorConnector::saveRobotPressed signal
      @param newRobotMsg [stdr_msgs::RobotMsg] The robot to be saved
      @param file_name [QString] The yaml file for the robot to be saved
      @return void
      **/
      void saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg,QString file_name);
      
      /**
      @brief Loads a robot from robot creator into map. Connects to the CGuiConnector::CRobotCreatorConnector::loadRobotPressed signal
      @param newRobotMsg [stdr_msgs::RobotMsg] The robot to be put in the map
      @return void
      **/
      void loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
      
      /**
      @brief Informs CGuiController that a robot is loaded from a yaml file. Connects to the CGuiConnector::robotFromFile signal
      @param newRobotMsg [stdr_msgs::RobotMsg] The robot to be put in the map
      @return void
      **/
      void loadRobotFromFilePressed(stdr_msgs::RobotMsg newRobotMsg);
      
      /**
      @brief Informs CGuiController that an RFID is going to be placed in the environment. Connects to the CGuiConnector::loadRfidPressed signal
      @return void
      **/
      void loadRfidPressed(void);
      
      /**
      @brief Informs CGuiController that a CO2 source is going to be placed in the environment. Connects to the CGuiConnector::loadCo2Pressed signal
      @return void
      **/
      void loadCo2Pressed(void);
      
      /**
      @brief Informs CGuiController that a sound source is going to be placed in the environment. Connects to the CGuiConnector::loadSoundPressed signal
      @return void
      **/
      void loadSoundPressed(void);
      
      /**
      @brief Informs CGuiController that a thermal source is going to be placed in the environment. Connects to the CGuiConnector::loadThermalPressed signal
      @return void
      **/
      void loadThermalPressed(void);
      
      /**
      @brief Performs zoom in when the button is pressed. Connects to the CMapConnector::zoomInPressed signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void zoomInPressed(QPoint p);
      
      /**
      @brief Performs zoom out when the button is pressed. Connects to the CMapConnector::zoomOutPressed signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void zoomOutPressed(QPoint p);
      
      /**
      @brief Gets the point at which the new robot is placed. Connects to the CMapConnector::robotPlaceSet signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void robotPlaceSet(QPoint p);
      
      /**
      @brief Gets the point at which the new RFID tag is placed. Connects to the CMapConnector::robotPlaceSet signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void rfidPlaceSet(QPoint p);
      
      /**
      @brief Gets the point at which the new thermal source is placed. Connects to the CMapConnector::thermalPlaceSet signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void thermalPlaceSet(QPoint p);
      
      /**
      @brief Gets the point at which the new CO2 source is placed. Connects to the CMapConnector::co2PlaceSet signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void co2PlaceSet(QPoint p);
      
      /**
      @brief Gets the point at which the new sound source is placed. Connects to the CMapConnector::soundPlaceSet signal
      @param p [QPoint] The event point in the OGM
      @return void
      **/
      void soundPlaceSet(QPoint p);
      
      /**
      @brief Updates the map to be shown in GUI. Connects to the timeout signal of timer_
      @return void
      **/
      void updateMapInternal(void);
      
      /**
      @brief Informs CGuiController that a laser visualizer has been clicked. Connects to the CInfoConnector::laserVisualizerClicked signal
      @param robotName [QString] Frame id of the robot
      @param laserName [QString] Frame id of the laser
      @return void
      **/
      void laserVisualizerClicked(QString robotName,QString laserName);
      
      /**
      @brief Informs CGuiController that a sonar visualizer has been clicked. Connects to the CInfoConnector::sonarVisualizerClicked signal
      @param robotName [QString] Frame id of the robot
      @param sonarName [QString] Frame id of the sonar
      @return void
      **/
      void sonarVisualizerClicked(QString robotName,QString sonarName);
      
      /**
      @brief Informs CGuiController that a robot visualizer has been clicked. Connects to the CInfoConnector::robotVisualizerClicked signal
      @param robotName [QString] Frame id of the robot
      @return void
      **/
      void robotVisualizerClicked(QString robotName);
      
      /**
      @brief Informs CGuiController that click has performed in the map. Connects to the CMapConnector::itemClicked signal
      @param p [QPoint] The event point in map
      @param b [Qt::MouseButton] The mouse button used to trigger the event
      @return void
      **/
      void itemClicked(QPoint p,Qt::MouseButton b);
      
      /**
      @brief Informs CGuiController about the new pose of a robot. Connects to the CMapConnector::robotReplaceSet signal
      @param p [QPoint] The event point in map
      @param robotName [std::string] The frame id of the re-placed robot
      @return void
      **/
      void robotReplaceSet(QPoint p,std::string robotName);
      
      /**
      @brief Informs CGuiController that a laser visibility status has been clicked. Connects to the CInfoConnector::laserVisibilityClicked signal
      @param robotName [QString] Frame id of the robot
      @param laserName [QString] Frame id of the laser
      @return void
      **/
      void laserVisibilityClicked(QString robotName,QString laserName);
      
      /**
      @brief Informs CGuiController that a sonar visibility status has been clicked. Connects to the CInfoConnector::sonarVisibilityClicked signal
      @param robotName [QString] Frame id of the robot
      @param sonarName [QString] Frame id of the sonar
      @return void
      **/
      void sonarVisibilityClicked(QString robotName,QString sonarName);
      
      /**
      @brief Informs CGuiController that a rfidReader visibility status has \
      been clicked. Connects to the CInfoConnector::rfidReaderVisibilityClicked\
      signal
      @param robotName [QString] Frame id of the robot
      @param rfidReaderName [QString] Frame id of the rfidReader
      @return void
      **/
      void rfidReaderVisibilityClicked(QString robotName,QString rfidReaderName);
      
      /**
      @brief Informs CGuiController that a CO2Sensor visibility status has \
      been clicked. Connects to the CInfoConnector::co2SensorVisibilityClicked\
      signal
      @param robotName [QString] Frame id of the robot
      @param co2SensorName [QString] Frame id of the co2 sensor
      @return void
      **/
      void co2SensorVisibilityClicked(QString robotName,QString co2SensorName);
      
      /**
      @brief Informs CGuiController that a Thermal Sensor visibility status has \
      been clicked. Connects to the CInfoConnector::thermalSensorVisibilityClicked\
      signal
      @param robotName [QString] Frame id of the robot
      @param thermalSensorName [QString] Frame id of the thermal sensor
      @return void
      **/
      void thermalSensorVisibilityClicked(QString robotName,QString thermalSensorName);
      
      /**
      @brief Informs CGuiController that a sound Sensor visibility status has \
      been clicked. Connects to the CInfoConnector::soundSensorVisibilityClicked\
      signal
      @param robotName [QString] Frame id of the robot
      @param soundSensorName [QString] Frame id of the sound sensor
      @return void
      **/
      void soundSensorVisibilityClicked(QString robotName,QString soundSensorName);
      
      /**
      @brief Informs CGuiController that a robot visibility status has been clicked. Connects to the CInfoConnector::robotVisibilityClicked signal
      @param robotName [QString] Frame id of the robot
      @return void
      **/
      void robotVisibilityClicked(QString robotName);
    
    //------------------------------------------------------------------------//
    Q_SIGNALS:
    
      /**
      @brief Is emitted when a new robot is going to be placed. Connects to the CMapConnector::waitForPlace slot
      @return void
      **/
      void waitForRobotPose(void);
      
      /**
      @brief Is emitted when a new thermal source is going to be placed. Connects to the CMapConnector::waitForThermalPose slot
      @return void
      **/
      void waitForThermalPose(void);
      
      /**
      @brief Is emitted when a new CO2 source is going to be placed. Connects to the CMapConnector::waitForCo2Pose slot
      @return void
      **/
      void waitForCo2Pose(void);
      
      /**
      @brief Is emitted when a new sound source is going to be placed. Connects to the CMapConnector::waitForSOundPose slot
      @return void
      **/
      void waitForSoundPose(void);
      
      /**
      @brief Is emitted when a new rfid tag is going to be placed. Connects to the CMapConnector::waitForRfidPose slot
      @return void
      **/
      void waitForRfidPose(void);
      
      /**
      @brief Is emitted when a robot is going to be re-placed. Connects to the CMapConnector::waitForReplace slot
      @param robotFrameId [std::string] The frame id of the robot to be re-placed
      @return void
      **/
      void replaceRobot(std::string robotFrameId);
      
      /**
      @brief Is emitted when a laser's visibility status is going to be changed. Connects to the CInfoConnector::setLaserVisibility slot
      @param robotName [QString] The frame id of the robot containing the specific laser
      @param laserName [QString] The frame id of the laser
      @param vs [char] The new visibility status
      @return void
      **/
      void setLaserVisibility(QString robotName,QString laserName,char vs);
      
      /**
      @brief Is emitted when a sonar's visibility status is going to be changed. Connects to the CInfoConnector::setSonarVisibility slot
      @param robotName [QString] The frame id of the robot containing the specific sonar
      @param sonarName [QString] The frame id of the laser
      @param vs [char] The new visibility status
      @return void
      **/
      void setSonarVisibility(QString robotName,QString sonarName,char vs);
      
      /**
      @brief Is emitted when a rfid sensor's visibility status is going to be \
      changed. Connects to the CInfoConnector::setRfidReaderVisibility slot
      @param robotName [QString] The frame id of the robot containing the specific rfid reader
      @param rfidReaderName [QString] The frame id of the rfidReader
      @param vs [char] The new visibility status
      @return void
      **/
      void setRfidReaderVisibility(QString robotName, 
        QString rfidReaderName, char vs);
        
      /**
      @brief Is emitted when a CO2 sensor's visibility status is going to be \
      changed. Connects to the CInfoConnector::setCO2SensorVisibility slot
      @param robotName [QString] The frame id of the robot containing the specific co2 reader
      @param co2SensorName [QString] The frame id of the CO2 sensor
      @param vs [char] The new visibility status
      @return void
      **/
      void setCO2SensorVisibility(QString robotName, 
        QString co2SensorName, char vs);
        
      /**
      @brief Is emitted when a thermal sensor's visibility status is going to be \
      changed. Connects to the CInfoConnector::setThermalSensorVisibility slot
      @param robotName [QString] The frame id of the robot containing the specific sensor
      @param thermalSensorName [QString] The frame id of the thermal sensor
      @param vs [char] The new visibility status
      @return void
      **/
      void setThermalSensorVisibility(QString robotName, 
        QString thermalSensorName, char vs);
        
      /**
      @brief Is emitted when a sound sensor's visibility status is going to be \
      changed. Connects to the CInfoConnector::setSoundSensorVisibility slot
      @param robotName [QString] The frame id of the robot containing the specific sensor
      @param soundSensorName [QString] The frame id of the sound sensor
      @param vs [char] The new visibility status
      @return void
      **/
      void setSoundSensorVisibility(QString robotName, 
        QString soundSensorName, char vs);
      
      /**
      @brief Is emitted when a robot's visibility status is going to be changed. Connects to the CInfoConnector::setRobotVisibility slot
      @param robotName [QString] The frame id of the robot
      @param vs [char] The new visibility status
      @return void
      **/
      void setRobotVisibility(QString robotName,char vs);
  };
}

#endif

