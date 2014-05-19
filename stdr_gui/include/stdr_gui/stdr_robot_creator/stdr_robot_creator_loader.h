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

#ifndef STDR_ROBOT_CREATOR_LOADER
#define STDR_ROBOT_CREATOR_LOADER

#include "ui_robotCreator.h"
#include "stdr_gui/stdr_robot_creator/stdr_kinematic_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_rfid_antenna_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_co2_sensor_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_thermal_sensor_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_sound_sensor_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_robot_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_robot_footprint_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_sonar_properties_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_laser_properties_loader.h"
#include "stdr_gui/stdr_tools.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CRobotCreatorLoader
  @brief Implements the low level functionalities of the robot creator. Inherits form QWidget and Ui_RobotCreator (auto created from ui file)
  **/ 
  class CRobotCreatorLoader : public QWidget, public Ui_RobotCreator
  {  
    //------------------------------------------------------------------------//
    private:
      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char **  argv_;
    //------------------------------------------------------------------------//
    public:
      //!< Holds the tree items that contain laser sensors
      std::vector<QTreeWidgetItem> lasers;
      //!< Holds the tree items that contain sonar sensors
      std::vector<QTreeWidgetItem> sonars;
      //!< Holds the tree items that contain rfid antenna sensors
      std::vector<QTreeWidgetItem> rfids;
      std::vector<QTreeWidgetItem> co2_sensors;
      std::vector<QTreeWidgetItem> thermal_sensors;
      std::vector<QTreeWidgetItem> sound_sensors;
      
      //!< Tree item for the robot
      QTreeWidgetItem robotNode;
      //!< Tree item for the lasers root
      QTreeWidgetItem lasersNode;
      //!< Tree item for the sonars root
      QTreeWidgetItem sonarsNode;
      //!< Tree item for the rfid antennas root
      QTreeWidgetItem rfidAntennasNode;
      QTreeWidgetItem co2SensorsNode;
      QTreeWidgetItem thermalSensorsNode;
      QTreeWidgetItem soundSensorsNode;
      //!< Tree item for the kinematic
      QTreeWidgetItem kinematicNode;
      //!< Tree item for the robot orientation
      QTreeWidgetItem robotInfoOrientation;
      //!< Tree item for the robot radius
      QTreeWidgetItem robotInfoRadius;
      
      //!< Tree item for the robot footprint
      QTreeWidgetItem robotInfoFootprint;
      //!< Holds the tree items that contain the footprint points
      std::vector<QTreeWidgetItem> footPoints;
      
      //!< Holds the robot preview image
      QImage robotPreviewImage;
      
      //!< Add icon
      QIcon   addIcon;
      //!< Edit icon
      QIcon  editIcon;
      //!< Remove icon
      QIcon  removeIcon;
      //!< Save icon
      QIcon  saveIcon;
      //!< Load icon
      QIcon  loadIcon;
          
      //!< Object of robot properties widget
      CRobotPropertiesLoader robotPropLoader;
      //!< Object of robot footprint widget
      CRobotFootprintLoader robotFootLoader;
      //!< Object of laser properties widget
      CLaserPropertiesLoader laserPropLoader;
      //!< Object of sonar properties widget
      CSonarPropertiesLoader sonarPropLoader;
      //!< Object of robot kinematic properties widget
      CKinematicPropertiesLoader kinematicPropLoader;
      //!< Object of rfid antenna properties widget
      CRfidAntennaPropertiesLoader rfidAntennaPropLoader;
      CCO2SensorPropertiesLoader co2SensorPropLoader;
      CThermalSensorPropertiesLoader thermalSensorPropLoader;
      CSoundSensorPropertiesLoader soundSensorPropLoader;
      
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char**] Input arguments
      @return void
      **/
      CRobotCreatorLoader(int argc, char **argv);
      
      /**
      @brief Sets up the information tree in robot creator widget
      @return void
      **/
      void setupInitialTree(void);
  };  
}

#endif
