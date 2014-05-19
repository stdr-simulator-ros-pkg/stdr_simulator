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
      @brief Virtual function for sensor value callback. Implement this function
      on derived class to publish sensor measurements.
      @return void
      **/ 
      virtual void updateSensorCallback(void) = 0;
      
      /**
      @brief Getter function for returning the sensor pose relatively to robot
      @return geometry_msgs::Pose2D
      **/ 
      inline geometry_msgs::Pose2D getSensorPose(void) const
      {
        return _sensorPose;
      }
      
      /**
      @brief Getter function for returning the sensor frame id
      @return std::string
      **/ 
      inline std::string getFrameId(void) const
      {
        return _namespace + "_" + _sensorFrameId;
      } 
      
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
      @param n [ros::NodeHandle& n] A ROS NodeHandle to create timers
      @param sensorPose [const geometry_msgs::Pose2D&] The sensor's pose relative to robot
      @param sensorFrameId [const std::string&] The sensor's frame id
      @param updateFrequency [float] The sensor's update frequnecy
      @return void
      **/ 
      Sensor(
            const nav_msgs::OccupancyGrid& map,
            const std::string& name,
            ros::NodeHandle& n,
            const geometry_msgs::Pose2D& sensorPose,
            const std::string& sensorFrameId,
            float updateFrequency);
      
      /**
      @brief Checks if transform is available and calls updateSensorCallback()
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/
      void checkAndUpdateSensor(const ros::TimerEvent& ev);
      
      /**
      @brief Function for updating the sensor tf transform
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/ 
      void updateTransform(const ros::TimerEvent& ev);
      
    protected:
    
      //!< The base for the sensor frame_id
      const std::string& _namespace;
      //!< The environment occupancy grid map
      const nav_msgs::OccupancyGrid& _map;
      
      //!< Sensor pose relative to robot
      const geometry_msgs::Pose2D _sensorPose;
      //!< Update frequency of _timer
      const float _updateFrequency;
      //!< Sensor frame id
      const std::string _sensorFrameId;
      
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
