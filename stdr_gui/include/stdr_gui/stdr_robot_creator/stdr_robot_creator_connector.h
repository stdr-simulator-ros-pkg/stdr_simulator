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

#ifndef STDR_ROBOT_CREATOR_CONNECTOR
#define STDR_ROBOT_CREATOR_CONNECTOR

#include "stdr_gui/stdr_robot_creator/stdr_robot_creator_loader.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CRobotCreatorConnector
  @brief Implements the high level functionalities of the robot creator widget. Inherits form QObject
  **/ 
  class CRobotCreatorConnector:
    public QObject
  {
    Q_OBJECT
    //------------------------------------------------------------------------//
    private:
      //!< Number of input arguments
      int   argc_; 
      //!< Input arguments
      char**  argv_;
      
      //!< Variable that holds the proportion coefficient for the visualization of robot creator image
      float climax_;
      
      //!< Object of CRobotCreatorLoader
      CRobotCreatorLoader loader_;
      
      //!< Container of the new robot message
      stdr_msgs::RobotMsg new_robot_msg_;
      
      //!< Current laser for highlight
      unsigned int laser_hightlight_id_;
      //!< Current sonar for highlight
      unsigned int sonar_hightlight_id_;
      //!< Current rfid antenna for highlight
      unsigned int rfid_antenna_hightlight_id_;
      //!< Current rfid antenna for highlight
      unsigned int co2_sensor_hightlight_id_;
      unsigned int thermal_sensor_hightlight_id_;
      unsigned int sound_sensor_hightlight_id_;
      
      //!< Tree item of the currently clicked laser 
      QTreeWidgetItem* current_laser_;
      //!< Tree item of the currently clicked sonar
      QTreeWidgetItem* current_sonar_;
      //!< Tree item of the currently clicked rfid
      QTreeWidgetItem* current_rfid_;
      QTreeWidgetItem* current_co2_sensor_;
      QTreeWidgetItem* current_thermal_sensor_;
      QTreeWidgetItem* current_sound_sensor_;
      //!< Tree item of the currently clicked footprint point
      QTreeWidgetItem* current_footprint_point_;
      
      /**
      @brief Pops up a message box with a specific message
      @param msg [QString] Message to be shown
      @return void
      **/
      void showMessage(QString msg);
    
    //------------------------------------------------------------------------//
    public:
    
      //!< Number of laser sensors
      static int laser_number;
      //!< Number of sonar sensors
      static int sonar_number;
      //!< Number of rfid antenna sensors
      static int rfid_number;
      static int co2_sensors_number;
      static int thermal_sensors_number;
      static int sound_sensors_number;
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char**] Input arguments
      @return void
      **/
      CRobotCreatorConnector(int argc, char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CRobotCreatorConnector(void);
      
      /**
      @brief Initializes the robot creator
      @return void
      **/
      void initialise(void);
      
      /**
      @brief Deletes a specific tree item and it's children recursively
      @param item [QTreeWidgetItem*] The item to be erased
      @return void
      **/
      void deleteTreeNode(QTreeWidgetItem *item);
      
      /**
      @brief Shows the edit robot widget
      @return void
      **/
      void editRobot(void);
      
      /**
      @brief Adds a laser sensor in the new robot 
      @return void
      **/
      void addLaser(void);
      
      /**
      @brief Adds a specific laser sensor in the new robot 
      @param lmsg [stdr_msgs::LaserSensorMsg] The laser sensor to be added
      @return void
      **/
      void addLaser(stdr_msgs::LaserSensorMsg lmsg);
      
      /**
      @brief Erases a specific laser sensor based on a tree item
      @param item [QTreeWidgetItem*] Tree item that holds the specific laser sensor 
      @return void
      **/
      void eraseLaser(QTreeWidgetItem *item);
      
      /**
      @brief Edits a specific laser sensor based on a tree item. Initiates the laser sensor editor widget
      @param item [QTreeWidgetItem*] Tree item that holds the specific laser sensor 
      @return void
      **/
      void editLaser(QTreeWidgetItem *item);
      
      /**
      @brief Returns the ID of a laser sensor
      @param frameId [QString] The frame id of the laser sensor 
      @return int
      **/
      int searchLaser(QString frameId);
      
      /**
      @brief Saves a specific laser sensor in a file
      @param item [QTreeWidgetItem*] The tree item that holds the laser sensor 
      @return void
      **/
      void saveLaser(QTreeWidgetItem *item);
      
      /**
      @brief Loads a specific laser sensor from a file
      @param item [QTreeWidgetItem*] The tree item that holds the laser sensor 
      @return void
      **/
      void loadLaser(QTreeWidgetItem *item);
      
      /**
      @brief Updates a tree item with a specific laser sensor
      @param item [QTreeWidgetItem*] The tree item that will be updated
      @param l [stdr_msgs::LaserSensorMsg] The laser sensor message
      @return void
      **/
      void updateLaserTree(QTreeWidgetItem *item,stdr_msgs::LaserSensorMsg l);
      
      /**
      @brief Adds a footprint point in the new robot 
      @return void
      **/
      void addFootprintPoint(void);
      
      /**
      @brief Adds a footprint point in the new robot 
      @return void
      **/
      void addFootprintPoint(geometry_msgs::Point pt);
      
      /**
      @brief Erases a footprint point in the new robot 
      @return void
      **/
      void eraseFootprintPoint(QTreeWidgetItem *item);
      
      /**
      @brief Adds a sonar sensor in the new robot 
      @return void
      **/
      void addSonar(void);
      
      /**
      @brief Adds a specific sonar sensor in the new robot 
      @param smsg [stdr_msgs::SonarSensorMsg] The sonar sensor to be added
      @return void
      **/
      void addSonar(stdr_msgs::SonarSensorMsg smsg);
      
      /**
      @brief Erases a specific sonar sensor based on a tree item
      @param item [QTreeWidgetItem*] Tree item that holds the specific sonar sensor 
      @return void
      **/
      void eraseSonar(QTreeWidgetItem *item);
      
      /**
      @brief Edits a specific sonar sensor based on a tree item. Initiates the sonar sensor editor widget
      @param item [QTreeWidgetItem*] Tree item that holds the specific sonar sensor 
      @return void
      **/
      void editSonar(QTreeWidgetItem *item);
      
      /**
      @brief Returns the ID of a sonar sensor
      @param frameId [QString] The frame id of the sonar sensor 
      @return int
      **/
      int searchSonar(QString frameId);
      
      /**
      @brief Saves a specific sonar sensor in a file
      @param item [QTreeWidgetItem*] The tree item that holds the sonar sensor 
      @return void
      **/
      void saveSonar(QTreeWidgetItem *item);
      
      /**
      @brief Saves a specific rfid reader sensor in a file
      @param item [QTreeWidgetItem*] The tree item that holds the sensor 
      @return void
      **/
      void saveRfidAntenna(QTreeWidgetItem *item);
      void saveCO2Sensor(QTreeWidgetItem *item);
      void saveThermalSensor(QTreeWidgetItem *item);
      void saveSoundSensor(QTreeWidgetItem *item);
      
      /**
      @brief Loads a specific sonar sensor from a file
      @param item [QTreeWidgetItem*] The tree item that holds the sonar sensor 
      @return void
      **/
      void loadSonar(QTreeWidgetItem *item);
      
      /**
      @brief Loads a specific rfid reader sensor from a file
      @param item [QTreeWidgetItem*] The tree item that holds the sensor 
      @return void
      **/
      void loadRfidAntenna(QTreeWidgetItem *item);
      void loadCO2Sensor(QTreeWidgetItem *item);
      void loadThermalSensor(QTreeWidgetItem *item);
      void loadSoundSensor(QTreeWidgetItem *item);
      
      /**
      @brief Updates a tree item with a specific sonar sensor
      @param item [QTreeWidgetItem*] The tree item that will be updated
      @param l [stdr_msgs::SonarSensorMsg] The sonar sensor message
      @return void
      **/
      void updateSonarTree(QTreeWidgetItem *item,stdr_msgs::SonarSensorMsg l);
      
      /**
      @brief Updates a tree item with a specific rfid reader sensor
      @param item [QTreeWidgetItem*] The tree item that will be updated
      @param l [stdr_msgs::RfidSensorMsg] The rfid reader sensor message
      @return void
      **/
      void updateRfidTree(QTreeWidgetItem *item,stdr_msgs::RfidSensorMsg l);
      void updateCO2SensorTree(QTreeWidgetItem *item,stdr_msgs::CO2SensorMsg l);
      void updateThermalSensorTree(QTreeWidgetItem *item,stdr_msgs::ThermalSensorMsg l);
      void updateSoundSensorTree(QTreeWidgetItem *item,stdr_msgs::SoundSensorMsg l);
      
      /**
      @brief Adds an rfid antenna sensor in the new robot 
      @return void
      **/
      void addRfidAntenna(void);
      void addCO2Sensor(void);
      void addThermalSensor(void);
      void addSoundSensor(void);
      
      /**
      @brief Adds an rfid antenna sensor in the new robot 
      @param rmsg [stdr_msgs::RfidSensorMsg] The rfid antenna sensor message
      @return void
      **/
      void addRfidAntenna(stdr_msgs::RfidSensorMsg rmsg);
      void addCO2Sensor(stdr_msgs::CO2SensorMsg rmsg);
      void addThermalSensor(stdr_msgs::ThermalSensorMsg rmsg);
      void addSoundSensor(stdr_msgs::SoundSensorMsg rmsg);
      
      /**
      @brief Erases a specific rfid antenna sensor based on a tree item
      @param item [QTreeWidgetItem*] Tree item that holds the specific rfid antenna sensor 
      @return void
      **/
      void eraseRfid(QTreeWidgetItem *item);
      void eraseCO2Sensor(QTreeWidgetItem *item);
      void eraseThermalSensor(QTreeWidgetItem *item);
      void eraseSoundSensor(QTreeWidgetItem *item);
      
      /**
      @brief Edits a specific rfid antenna sensor based on a tree item. Initiates the rfid antenna sensor editor widget
      @param item [QTreeWidgetItem*] Tree item that holds the specific rfid antenna sensor 
      @return void
      **/
      void editRfid(QTreeWidgetItem *item);
      void editCO2Sensor(QTreeWidgetItem *item);
      void editThermalSensor(QTreeWidgetItem *item);
      void editSoundSensor(QTreeWidgetItem *item);
      
      /**
      @brief Edits a specific footprint point based on a tree item. Initiates the footprint editor widget
      @param item [QTreeWidgetItem*] Tree item that holds the specific footprint point 
      @return void
      **/
      void editFootprintPoint(QTreeWidgetItem *item);
      
      /**
      @brief Returns the ID of an rfid antenna sensor
      @param frameId [QString] The frame id of the rfid antenna sensor 
      @return int
      **/
      int searchRfid(QString frameId);
      int searchCO2Sensor(QString frameId);
      int searchThermalSensor(QString frameId);
      int searchSoundSensor(QString frameId);
      
      /**
      @brief Draws a robot
      @return void
      **/
      void drawRobot(void);
      
      /**
      @brief Draws the robot's lasers
      @return void
      **/
      void drawLasers(void);
      
      /**
      @brief Draws the robot's sonars
      @return void
      **/
      void drawSonars(void);
      
      /**
      @brief Draws the robot's rfid antennas
      @return void
      **/
      void drawRfidAntennas(void);
      void drawCO2Sensors(void);
      void drawThermalSensors(void);
      void drawSoundSensors(void);

      /**
      @brief Updates the robot's preview
      @return void
      **/
      void updateRobotPreview(void);
      
      /**
      @brief Updates the robot's tree widget
      @return void
      **/
      void updateRobotTree(void);
      
      /**
      @brief Sets the robot's initial pose
      @param x [float] x coordinate
      @param y [float] y coordinate
      @return void
      **/
      void setInitialPose(float x, float y);
      
      /**
      @brief Returns the created robot
      @return stdr_msgs::RobotMsg : The new robot
      **/
      stdr_msgs::RobotMsg getNewRobot(void);
      
      /**
      @brief Sets the created robot
      @param msg [stdr_msgs::RobotMsg] The new robot
      @return void
      **/
      void setNewRobot(stdr_msgs::RobotMsg msg);
    
    //------------------------------------------------------------------------//  
    public Q_SLOTS:
    
      /**
      @brief Called when a tree item is clicked
      @param item [QTreeWidgetItem *] The item clicked
      @param column [int] The column clicked
      @return void
      **/
      void treeItemClicked ( QTreeWidgetItem * item, int column );
      
      /**
      @brief Called when the update button of the properties widget is clicked 
      @return void
      **/ 
      void updateLaser(void);
      
      /**
      @brief Called when the update button of the properties widget is clicked 
      @return void
      **/ 
      void updateSonar(void);
      
      /**
      @brief Called when the refresh button of the properties widget is clicked 
      @return void
      **/ 
      void updateLaserOpen(void);
      
      /**
      @brief Called when the refresh button of the properties widget is clicked 
      @return void
      **/ 
      void updateSonarOpen(void);
      
      /**
      @brief Called when the update button of the properties widget is clicked 
      @return void
      **/ 
      void updateRfidAntenna(void);
      void updateCO2Sensor(void);
      void updateThermalSensor(void);
      void updateSoundSensor(void);
      void updateKinematicModel(void);
      
      /**
      @brief Called when the update button of the properties widget is clicked 
      @return void
      **/ 
      void updateRfidAntennaOpen(void);
      void updateCO2SensorOpen(void);
      void updateThermalSensorOpen(void);
      void updateSoundSensorOpen(void);
      
      /**
      @brief Called when the update button of the properties widget is clicked 
      @return void
      **/ 
      void updateRobot(void);
      
      /**
      @brief Called when the refresh button of the properties widget is clicked 
      @return void
      **/ 
      void updateRobotOpen(void);
      
      /**
      @brief Called when the update button of the footprint widget is clicked 
      @return void
      **/ 
      void updateFootprintPoint(void);
      
      /**
      @brief Called when the refresh button of the properties widget is clicked 
      @return void
      **/ 
      void updateFootprintPointOpen(void);
      
      /**
      @brief Called when the save robot button is clicked 
      @return void
      **/ 
      void saveRobot(void);
      
      /**
      @brief Called when the load robot button 
      @return void
      **/ 
      void getRobotFromYaml(void);
      
      /**
      @brief Called when the load robot in map button is clicked 
      @return void
      **/ 
      void loadRobot(void);
    
    //------------------------------------------------------------------------//  
    Q_SIGNALS:
    
      /**
      @brief Emitted when the "load robot in map" button is pressed
      @param newRobotMsg [stdr_msgs::RobotMsg] The new robot to be placed in the map
      @return void
      **/ 
      void loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
      
      /**
      @brief Emitted to save the robot in a yaml file
      @param newRobotMsg [stdr_msgs::RobotMsg] The new robot to be saved
      @param file_name [QString] The file name for the robot description to be saved
      @return void
      **/ 
      void saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg,QString file_name);
  };
}

#endif
