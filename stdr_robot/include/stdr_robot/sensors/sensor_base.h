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

#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.141592653589793

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {

  /**
  @class Sensor
  @brief A class that provides sensor abstraction
  **/ 
  class Sensor {
    
    public:
      
      /**
      @brief Pure virtual function for sensor value callback
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/ 
      virtual void updateSensorCallback(const ros::TimerEvent& ev) = 0;
      
      /**
      @brief Pure virtual function for returning the sensor pose relatively to robot
      @return geometry_msgs::Pose2D
      **/ 
      virtual geometry_msgs::Pose2D getSensorPose(void) = 0;
      
      /**
      @brief Pure virtual function for returning the sensor frame id
      @return std::string
      **/ 
      virtual std::string getFrameId(void) = 0; 
      
      /**
      @brief Pure virtual function for updating the sensor tf transform
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/ 
      virtual void updateTransform(const ros::TimerEvent& ev) = 0;
      
      /**
      @brief Default destructor
      @return void
      **/ 
      virtual ~Sensor(void) 
      {
      }
    
    protected:
      
      /**
      @brief Default constructor
      @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
      @param name [const std::string&] The sensor frame id without the base
      @return void
      **/ 
      Sensor(const nav_msgs::OccupancyGrid& map,const std::string& name): 
        _map(map), 
        _namespace(name), 
        _gotTransform(false) 
      {
      }
      
    protected:
    
      //!< The base for the sensor frame_id
      const std::string& _namespace;
      //!< The environment occupancy grid map
      const nav_msgs::OccupancyGrid& _map;
      
      //!< A ROS timer for updating the sensor values
      ros::Timer _timer;
      //!< A ROS timer for updating the sensor tf
      ros::Timer _tfTimer;
      
      //!< ROS publisher for posting the sensor measurements
      ros::Publisher _publisher;
      //!< ROS tf listener
      tf::TransformListener _tfListener;
      //!< ROS transform from sensor to map
      tf::StampedTransform _sensorTransform;
      
      //!< True if sensor got the _sensorTransform
      bool _gotTransform;
  };

  typedef boost::shared_ptr<Sensor> SensorPtr;
  typedef std::vector<SensorPtr> SensorPtrVector;
    
}

#endif
