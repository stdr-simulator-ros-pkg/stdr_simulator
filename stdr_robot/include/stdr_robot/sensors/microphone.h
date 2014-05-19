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

#ifndef MICROPHONE_SENSOR_H
#define MICROPHONE_SENSOR_H

#include <stdr_robot/sensors/sensor_base.h>
#include <stdr_robot/sensors/helper.h>
#include <stdr_msgs/SoundSensorMsg.h>
#include <stdr_msgs/SoundSensorMeasurementMsg.h>
#include <stdr_msgs/SoundSourceVector.h>

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {

  /**
  @class Sonar
  @brief A class that provides sound sensor implementation. \
  Inherits publicly Sensor
  **/ 
  class SoundSensor : public Sensor {

    public:
      /**
      @brief Default constructor
      @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
      @param msg [const stdr_msgs::SoundSensorMsg&] The sound sensor \
      description message
      @param name [const std::string&] The sensor frame id without the base
      @param n [ros::NodeHandle&] The ROS node handle
      @return void
      **/ 
      SoundSensor(
        const nav_msgs::OccupancyGrid& map,
        const stdr_msgs::SoundSensorMsg& msg, 
        const std::string& name, 
        ros::NodeHandle& n);
      
      /**
      @brief Updates the sensor measurements
      @return void
      **/ 
      virtual void updateSensorCallback();
      
      /**
      @brief Default destructor
      @return void
      **/ 
      ~SoundSensor(void);
      
      /**
      @brief Receives the existent sound sources
      @param msg [const stdr_msgs::SoundSourceVector&] The sound sources message
      @return void
      **/
      void receiveSoundSources(const stdr_msgs::SoundSourceVector& msg);

    private:

      //!< sound sensor description
      stdr_msgs::SoundSensorMsg _description;
      
      //!< ROS subscriber for sound sources
      ros::Subscriber sound_sources_subscriber_;
      
      //!< The currently existent sources
      stdr_msgs::SoundSourceVector sound_sources_;
  };

}

#endif
