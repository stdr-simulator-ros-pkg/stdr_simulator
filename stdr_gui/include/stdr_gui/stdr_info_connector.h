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

#ifndef STDR_INFO_CONNECTOR
#define STDR_INFO_CONNECTOR

#include "stdr_gui/stdr_info_loader.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CInfoConnector
  @brief Serves the Qt events of the tree-like information widget. Inherits from QObject
  **/ 
  class CInfoConnector : 
    public QObject
  {
    Q_OBJECT
    //------------------------------------------------------------------------//
    private:
      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char**  argv_;
    
    //------------------------------------------------------------------------//
    public:
    
      //!< Object of CInfoLoader
      CInfoLoader loader;
      
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CInfoConnector(int argc, char **argv);
      
      /**
      @brief Updates the information tree according to the ensemble of robots
      @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The existent robots
      @return void
      **/
      void updateTree(const stdr_msgs::RobotIndexedVectorMsg& msg);
      
      /**
      @brief Updates the information tree according to the specific map
      @param width [float] The map width
      @param height [float] The map height
      @param ocgd [float] The map resolution (m/pixel)
      @return void
      **/
      void updateMapInfo(float width,float height,float ocgd);
      
      /**
      @brief Returns the CInfoLoader object
      @return QWidget*
      **/
      QWidget* getLoader(void);

    //------------------------------------------------------------------------//
    public Q_SLOTS:
    
      /**
      @brief Called when a click occurs in the tree
      @param item [QTreeWidgetItem*] Item clicked
      @param column [int] Column clicked
      @return void
      **/
      void treeItemClicked ( QTreeWidgetItem * item, int column ); 
      
      /**
      @brief Adapts the columns width according to what is visible when an item is clicked
      @param item [QTreeWidgetItem*] Item clicked
      @param column [int] Column clicked
      @return void
      **/
      void adaptColumns(QTreeWidgetItem *item, int column);
      
      /**
      @brief Adapts the columns width according to what is visible when an item expands or collapses
      @param item [QTreeWidgetItem*] Item expanded / collapsed
      @return void
      **/
      void adaptColumns(QTreeWidgetItem *item);
      
      /**
      @brief Adapts the columns width according to what is visible. Called when adaptSignal is emmited
      @return void
      **/
      void adaptSlot(void);

      /**
      @brief Changes a laser visibility icon
      @param robotName [QString] The robot frame id
      @param laserName [QString] The laser frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setLaserVisibility(QString robotName,QString laserName,char vs);
      
      /**
      @brief Changes a sonar visibility icon
      @param robotName [QString] The robot frame id
      @param sonarName [QString] The sonar frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setSonarVisibility(QString robotName,QString sonarName,char vs);
      
      /**
      @brief Changes a rfid reader visibility icon
      @param robotName [QString] The robot frame id
      @param rfidReaderName [QString] The rfidReader frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setRfidReaderVisibility(QString robotName,
        QString rfidReaderName, char vs);
        
      /**
      @brief Changes a co2 sensor visibility icon
      @param robotName [QString] The robot frame id
      @param co2SensorName [QString] The co2Sensor frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setCO2SensorVisibility(QString robotName,
        QString co2SensorName, char vs);
        
      /**
      @brief Changes a thermal sensor visibility icon
      @param robotName [QString] The robot frame id
      @param thermalSensorName [QString] The thermalSensor frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setThermalSensorVisibility(QString robotName,
        QString thermalSensorName, char vs);
        
      /**
      @brief Changes a sound sensor visibility icon
      @param robotName [QString] The robot frame id
      @param soundSensorName [QString] The soundSensor frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setSoundSensorVisibility(QString robotName,
        QString soundSensorName, char vs);
      
      /**
      @brief Changes a robot visibility icon
      @param robotName [QString] The robot frame id
      @param vs [char] The visibility state
      @return void
      **/
      void setRobotVisibility(QString robotName,char vs);
      
    //------------------------------------------------------------------------//
    Q_SIGNALS:
    
      /**
      @brief Emmited when a laser visualizer is clicked
      @param robotName [QString] The robot frame id
      @param laserName [QString] The laser frame id
      @return void
      **/
      void laserVisualizerClicked(QString robotName,QString laserName);
      
      /**
      @brief Emmited when a sonar visualizer is clicked
      @param robotName [QString] The robot frame id
      @param sonarName [QString] The sonar frame id
      @return void
      **/
      void sonarVisualizerClicked(QString robotName,QString sonarName);
      
      /**
      @brief Emmited when a robot visualizer is clicked
      @param robotName [QString] The robot frame id
      @return void
      **/
      void robotVisualizerClicked(QString robotName);
      
      /**
      @brief Emmited when a laser visibility icon is clicked
      @param robotName [QString] The robot frame id
      @param laserName [QString] The laser frame id
      @return void
      **/
      void laserVisibilityClicked(QString robotName,QString laserName);
      
      /**
      @brief Emmited when a sonar visibility icon is clicked
      @param robotName [QString] The robot frame id
      @param sonarName [QString] The sonar frame id
      @return void
      **/
      void sonarVisibilityClicked(QString robotName,QString sonarName);
      
      /**
      @brief Emmited when a rfid reader visibility icon is clicked
      @param robotName [QString] The robot frame id
      @param rfidReaderName [QString] The rfid reader frame id
      @return void
      **/
      void rfidReaderVisibilityClicked(
        QString robotName, QString rfidReaderName);
        
      /**
      @brief Emmited when a co2 sensor visibility icon is clicked
      @param robotName [QString] The robot frame id
      @param co2SensorName [QString] The co2 sensor frame id
      @return void
      **/
      void co2SensorVisibilityClicked(
        QString robotName, QString co2SensorName);
        
      /**
      @brief Emmited when a thermal sensor visibility icon is clicked
      @param robotName [QString] The robot frame id
      @param thermalSensorName [QString] The thermal sensor frame id
      @return void
      **/
      void thermalSensorVisibilityClicked(
        QString robotName, QString thermalSensorName);
        
      /**
      @brief Emmited when a sound sensor visibility icon is clicked
      @param robotName [QString] The robot frame id
      @param soundSensorName [QString] The sound sensor frame id
      @return void
      **/
      void soundSensorVisibilityClicked(
        QString robotName, QString soundSensorName);
      
      /**
      @brief Emmited when a robot visibility icon is clicked
      @param robotName [QString] The robot frame id
      @return void
      **/
      void robotVisibilityClicked(QString robotName);
      
      /**
      @brief Emmited when resize of columns according to whats visible is needed
      @return void
      **/
      void adaptSignal(void);
  };
}

#endif
