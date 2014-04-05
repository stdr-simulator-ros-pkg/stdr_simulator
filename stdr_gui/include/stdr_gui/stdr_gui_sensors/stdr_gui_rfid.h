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

#ifndef STDR_GUI_RFID_CONTAINER
#define STDR_GUI_RFID_CONTAINER

#include "stdr_gui/stdr_tools.h"
#include "stdr_msgs/RfidSensorMsg.h"
#include "stdr_msgs/RfidSensorMeasurementMsg.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CGuiRfid
  @brief Implements the functionalities for an RFID antenna sensor
  **/ 
  class CGuiRfid
  {
    //------------------------------------------------------------------------//
    private:
    
      //!< The topic from which the new RFID tags will be got
      std::string topic_;
      //!< The description for the rfid antenna message
      stdr_msgs::RfidSensorMsg msg_;
      //!< A ros subscriber
      ros::Subscriber subscriber_;
      //!< Used to avoid drawing when a new sonar message arives
      bool lock_;
      //!< The ROS tf frame
      std::string tf_frame_;
      //!< Visualization status of the specific sonar
      char visualization_status_;
      //!< The stdr rfid sensor measurement msg
      stdr_msgs::RfidSensorMeasurementMsg tags_;
      
    //------------------------------------------------------------------------//
    public:
      
      /**
      @brief Default contructor
      @param msg [stdr_msgs::RfidSensorMsg] The rfid antenna description msg
      @param baseTopic [std::string] The ros topic for subscription
      @return void
      **/
      CGuiRfid(stdr_msgs::RfidSensorMsg msg,std::string baseTopic);
      
      /**
      @brief Callback for the rfid measurement message
      @param msg [const stdr_msgs::RfidSensorMeasurementMsg&] The new rfid\
       sensor measurement message
      @return void
      **/
      void callback(const stdr_msgs::RfidSensorMeasurementMsg& msg); 
      
      /**
      @brief Paints the rfid measurements in the map image
      @param m [QImage*] The image to be drawn
      @param ocgd [float] The map's resolution
      @param listener [tf::TransformListener *] ROS tf transform listener
      @return void
      **/
      void paint(QImage *m,float ocgd,tf::TransformListener *listener);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiRfid(void);
  };  
}

#endif
