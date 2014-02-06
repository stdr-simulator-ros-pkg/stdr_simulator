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

#ifndef STDR_SONAR_VISUALIZATION
#define STDR_SONAR_VISUALIZATION

#include "stdr_gui/stdr_tools.h"
#include "ui_sonarVisualization.h"
#include "sensor_msgs/Range.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CSonarVisualisation
  @brief Implements the functionalities of the sonar visualization widget. Inherits form QWidget and Ui_sonarVisualization (auto created from ui file)
  **/ 
  class CSonarVisualisation : 
    public QWidget, 
    public Ui_sonarVisualization
  {
    //------------------------------------------------------------------------//
    private:
      //!< True if the visualizer is active
      bool active_;
      //!< The map resolution
      float resolution_;
      //!< The latest sonar range
      sensor_msgs::Range   range_;
      //!< Subscriber for getting the sonar ranges
      ros::Subscriber   subscriber_;
      //!< Description of the sonar sensor
      stdr_msgs::SonarSensorMsg msg_;
      //!< The laser frame id
      QString name_;
    //------------------------------------------------------------------------//
    public:
      /**
      @brief Default contructor
      @param name [QString] Sonar frame id
      @param resolution [float] Map resolution
      @return void
      **/
      CSonarVisualisation(QString name,float resolution);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CSonarVisualisation(void);
    
      /**
      @brief Returns true if the visualizer is active
      @return bool
      **/
      bool getActive(void);
      
      /**
      @brief Sets the sonar description message
      @param msg [stdr_msgs::SonarSensorMsg] The sonar description
      @return void
      **/
      void setSonar(stdr_msgs::SonarSensorMsg msg);
      
      /**
      @brief Destroys the visualizer
      @return void
      **/
      void destruct(void);
      
      /**
      @brief Called when the close event is triggered
      @param event [QCloseEvent*] The close event
      @return void
      **/
      void closeEvent(QCloseEvent *event);
      
      /**
      @brief Called when new laser data are available
      @param msg [const sensor_msgs::Range&] The new sonar data
      @return void
      **/
      void callback(const sensor_msgs::Range& msg); 
      
      /**
      @brief Paints the visualizer
      @return void
      **/
      void paint(void);
  };  
}

#endif
