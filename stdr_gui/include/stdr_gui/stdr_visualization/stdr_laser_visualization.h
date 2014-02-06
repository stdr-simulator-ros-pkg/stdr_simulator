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

#ifndef STDR_LASER_VISUALIZATION
#define STDR_LASER_VISUALIZATION

#include "stdr_gui/stdr_tools.h"
#include "ui_laserVisualization.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CLaserVisualisation
  @brief Implements the functionalities of the laser visualization widget. Inherits form QWidget and Ui_laserVisualization (auto created from ui file)
  **/ 
  class CLaserVisualisation : 
    public QWidget, 
    public Ui_laserVisualization
  {
    //------------------------------------------------------------------------//
    private:
      
      //!< True if the visualizer is active
      bool active_;
      //!< The map resolution
      float resolution_;
      
      //!< The latest laser scan
      sensor_msgs::LaserScan   scan_;
      //!< Subscriber for getting the laser scans
      ros::Subscriber     subscriber_;
      
      //!< Description of the laser sensor
      stdr_msgs::LaserSensorMsg msg_;
      
      //!< The image to draw into
      QImage internal_image_;
      //!< A void image
      QImage void_image_;
      //!< The laser frame id
      QString name_;
    
    //------------------------------------------------------------------------//
    public:
      /**
      @brief Default contructor
      @param name [QString] Laser frame id
      @param resolution [float] Map resolution
      @return void
      **/
      CLaserVisualisation(QString name,float resolution);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CLaserVisualisation(void);
    
      /**
      @brief Returns true if the visualizer is active
      @return bool
      **/
      bool getActive(void);
      
      /**
      @brief Sets the laser description message
      @param msg [stdr_msgs::LaserSensorMsg] The laser description
      @return void
      **/
      void setLaser(stdr_msgs::LaserSensorMsg msg);
      
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
      @param msg [const sensor_msgs::LaserScan&] The new laser data
      @return void
      **/
      void callback(const sensor_msgs::LaserScan& msg); 
      
      /**
      @brief Paints the visualizer
      @return void
      **/
      void paint(void);
  };  
}

#endif
