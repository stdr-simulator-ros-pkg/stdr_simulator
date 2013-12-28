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

#ifndef STDR_GUI_LASER_CONTAINER
#define STDR_GUI_LASER_CONTAINER

#include "stdr_gui/stdr_tools.h"
#include "stdr_msgs/LaserSensorMsg.h"
#include "sensor_msgs/LaserScan.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CGuiLaser
  @brief Implements the functionalities for a laser sensor
  **/ 
  class CGuiLaser
  {
    //------------------------------------------------------------------------//
    private:
      
      //!< Used to avoid drawing when a new laser message arives
      bool lock_;
      //!< The ROS topic to which the subscription must be made for the new values to be taken
      std::string topic_;
      
      //!< The ROS tf frame
      std::string tf_frame_;
      
      //!< A laser sensor message : Depscription of a laser sensor
      stdr_msgs::LaserSensorMsg msg_;
      
      //!< Subscriber for the ros laser msg
      ros::Subscriber subscriber_;
      
      //!< The ros laser scan msg
      sensor_msgs::LaserScan scan_;
      
      //!< Visualization status of the specific laser
      char visualization_status_;
      
    //------------------------------------------------------------------------//
    public:
    
      /**
      @brief Default contructor
      @param msg [stdr_msgs::LaserSensorMsg] The laser description msg
      @param baseTopic [std::string] The ros topic for subscription
      @return void
      **/
      CGuiLaser(stdr_msgs::LaserSensorMsg msg,std::string baseTopic);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiLaser(void);
      
      /**
      @brief Callback for the ros laser message
      @param msg [const sensor_msgs::LaserScan&] The new laser scan message
      @return void
      **/
      void callback(const sensor_msgs::LaserScan& msg); 
      
      /**
      @brief Paints the laser scan in the map image
      @param m [QImage*] The image to be drawn
      @param ocgd [float] The map's resolution
      @param listener [tf::TransformListener *] ROS tf transform listener
      @return void
      **/
      void paint(QImage *m,float ocgd,tf::TransformListener *listener);
      
      /**
      @brief Paints the laser scan in it's own visualizer
      @param m [QImage*] The image to be drawn
      @param ocgd [float] The map's resolution
      @param maxRange [float] The maximum range of all the robot sensors. Used for the visualizer proportions 
      @return void
      **/
      void visualizerPaint(QImage *m,float ocgd,float maxRange);
      
      /**
      @brief Returns the max range of the specific laser sensor
      @return void
      **/
      float getMaxRange(void);
      
      /**
      @brief Returns the visibility status of the specific laser sensor
      @return void
      **/
      char getVisualizationStatus(void);
      
      /**
      @brief Toggles the visibility status of the specific laser sensor
      @return void
      **/
      void toggleVisualizationStatus(void);
      
      /**
      @brief Sets the visibility status of the specific laser sensor
      @param vs [char] The new visibility status
      @return void
      **/
      void setVisualizationStatus(char vs);
      
      /**
      @brief Returns the frame id of the specific laser sensor
      @return std::string : The laser frame id
      **/
      std::string getFrameId(void);
  };  
}

#endif
