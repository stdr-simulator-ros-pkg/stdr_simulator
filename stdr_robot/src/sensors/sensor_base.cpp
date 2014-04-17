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

#include <stdr_robot/sensors/sensor_base.h>

namespace stdr_robot {

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
  Sensor::Sensor(
      const nav_msgs::OccupancyGrid& map,
      const std::string& name,
      ros::NodeHandle& n,
      const geometry_msgs::Pose2D& sensorPose,
      const std::string& sensorFrameId,
      float updateFrequency)
      : 
        _map(map), 
        _namespace(name),
        _sensorPose(sensorPose),
        _sensorFrameId(sensorFrameId),
        _updateFrequency(updateFrequency),
        _gotTransform(false) 
  {
    _timer = n.createTimer(
      ros::Duration(1/updateFrequency), &Sensor::checkAndUpdateSensor, this);
    _tfTimer = n.createTimer(
      ros::Duration(1/(2*updateFrequency)), &Sensor::updateTransform, this);
  }

  
   /**
  @brief Checks if transform is available and calls updateSensorCallback()
  @param ev [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void Sensor::checkAndUpdateSensor(const ros::TimerEvent& ev)
  {
    if (!_gotTransform) { //!< wait for transform 
      return;
    }
    
    updateSensorCallback();
  }
  
  /**
  @brief Updates the sensor tf transform
  @return void
  **/ 
  void Sensor::updateTransform(const ros::TimerEvent&)
  {
    try {
      _tfListener.waitForTransform("map_static",
                                  _namespace + "_" + _sensorFrameId,
                                  ros::Time(0),
                                  ros::Duration(0.2));
      _tfListener.lookupTransform("map_static",
                                  _namespace + "_" + _sensorFrameId,
                                  ros::Time(0), _sensorTransform);
      _gotTransform = true;
    }
    catch (tf::TransformException ex) {
      ROS_DEBUG("%s",ex.what());
    }
  }
}  // namespace stdr_robot
