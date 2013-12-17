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

#ifndef SONAR_H
#define SONAR_H

#include <stdr_robot/sensors/sensor_base.h>
#include <sensor_msgs/Range.h>
#include <stdr_msgs/SonarSensorMsg.h>

namespace stdr_robot {

  class Sonar : public Sensor {

    public:

      Sonar(const nav_msgs::OccupancyGrid& map,
          const geometry_msgs::Pose2DPtr& robotPosePtr,
          tf::TransformBroadcaster& tf,
          const stdr_msgs::SonarSensorMsg& msg, 
          const std::string& name, 
          ros::NodeHandle& n);
      virtual void updateSensorCallback(const ros::TimerEvent&);
      virtual void tfCallback(const ros::TimerEvent&);

      ~Sonar() {}

    private:

      stdr_msgs::SonarSensorMsg _description;
  };

}

#endif
