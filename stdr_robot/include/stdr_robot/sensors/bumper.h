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

#ifndef BUMPER_H
#define BUMPER_H

#include <stdr_robot/sensors/sensor_base.h>
#include <stdr_msgs/Bumper.h>
#include <stdr_msgs/BumperSensorMsg.h>

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {

  /**
  @class Bumper
  @brief A class that provides bumper implementation. Inherits publicly Sensor
  **/ 
  class Bumper : public Sensor {

    public:
      /**
      @brief Default constructor
      @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
      @param msg [const stdr_msgs::BumperSensorMsg&] The bumper description message
      @param name [const std::string&] The sensor frame id without the base
      @param n [ros::NodeHandle&] The ROS node handle
      @return void
      **/ 
      Bumper(const nav_msgs::OccupancyGrid& map,
        const stdr_msgs::BumperSensorMsg& msg, 
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
      ~Bumper(void);

    private:

      //!< Bumper sensor description
      stdr_msgs::BumperSensorMsg _description;

      //!< Current status
      bool _contact;
      
  };

}

#endif
