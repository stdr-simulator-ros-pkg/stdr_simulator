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

#ifndef LASER_H
#define LASER_H

#include <stdr_robot/sensors/sensor_base.h>
#include <sensor_msgs/LaserScan.h>
#include <stdr_msgs/LaserSensorMsg.h>

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {
  
  /**
  @class Laser
  @brief A class that provides laser implementation. Inherits publicly Sensor
  **/ 
  class Laser : public Sensor {

    public:

      /**
      @brief Default constructor
      @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
      @param msg [const stdr_msgs::LaserSensorMsg&] The laser description message
      @param name [const std::string&] The sensor frame id without the base
      @param n [ros::NodeHandle&] The ROS node handle
      @return void
      **/ 
      Laser(const nav_msgs::OccupancyGrid& map,
        const stdr_msgs::LaserSensorMsg& msg, 
        const std::string& name, 
        ros::NodeHandle& n);
        
      /**
      @brief Updates the sensor measurements
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/ 
      virtual void updateSensorCallback(const ros::TimerEvent& ev);
      
      /**
      @brief Returns the sensor pose relatively to robot
      @return geometry_msgs::Pose2D
      **/ 
      virtual geometry_msgs::Pose2D getSensorPose();
      
      /**
      @brief Returns the sensor frame id
      @return std::string
      **/ 
      virtual std::string getFrameId();
      
      /**
      @brief Updates the sensor tf transform
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/ 
      virtual void updateTransform(const ros::TimerEvent&);
      
      /**
      @brief Default destructor
      @return void
      **/ 
      ~Laser() {}

    private:

      //!< Laser sensor description
      stdr_msgs::LaserSensorMsg _description;
  };

}

#endif
