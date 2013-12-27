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

namespace stdr_robot {

class Sensor {
  
  public:
    
    virtual void updateSensorCallback(const ros::TimerEvent&) = 0;
    virtual geometry_msgs::Pose2D getSensorPose() = 0;
    virtual std::string getFrameId() = 0; 
    virtual ~Sensor() {}
  
  protected:
    
    Sensor(const nav_msgs::OccupancyGrid& map,
        const geometry_msgs::Pose2DPtr& robotPosePtr,
        const std::string& name) 
    : _map(map), _namespace(name), _robotPosePtr(robotPosePtr) {}
    
  protected:
  
    const std::string& _namespace;
    const nav_msgs::OccupancyGrid& _map;
    const geometry_msgs::Pose2DPtr& _robotPosePtr;
    ros::Timer _timer;
    ros::Timer _tfTimer;
    ros::Publisher _publisher;
    tf::TransformListener _tfListener;
};

typedef boost::shared_ptr<Sensor> SensorPtr;
typedef std::vector<SensorPtr> SensorPtrVector;
  
}

#endif
