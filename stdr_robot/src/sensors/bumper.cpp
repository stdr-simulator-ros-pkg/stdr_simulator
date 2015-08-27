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

#include <stdr_robot/sensors/bumper.h>

namespace stdr_robot {

  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::BumperSensorMsg&] The bumper description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/ 
  Bumper::Bumper(const nav_msgs::OccupancyGrid& map,
      const stdr_msgs::BumperSensorMsg& msg, 
      const std::string& name,
      ros::NodeHandle& n)
  : 
    Sensor(map, name, n, msg.pose, msg.frame_id, msg.frequency)
  {
    _description = msg;

    _publisher = n.advertise<stdr_msgs::Bumper>
	( _namespace + "/" + msg.frame_id, 1 );
  }
  
  /**
  @brief Default destructor
  @return void
  **/ 
  Bumper::~Bumper(void)
  {
    
  }

  /**
  @brief Updates the sensor measurements
  @return void
  **/ 
  void Bumper::updateSensorCallback() 
  {
    float angle;
    int distance;
    int xMap, yMap;
    stdr_msgs::Bumper bumperMsg;

    if ( _map.info.height == 0 || _map.info.width == 0 )
    {
      ROS_DEBUG("Outside limits\n");
      return;
    }

    // check if the bumper origin hits the occupied grid
    distance = 5;
    xMap = _sensorTransform.getOrigin().x() / _map.info.resolution + 
	cos(tf::getYaw(_sensorTransform.getRotation())) * distance;
    yMap = _sensorTransform.getOrigin().y() / _map.info.resolution + 
	sin(tf::getYaw(_sensorTransform.getRotation())) * distance;

    if (yMap * _map.info.width + xMap > _map.info.height*_map.info.width) {
	return;
    }
    //!< Found obstacle
    bumperMsg.contact = false;
    if ( _map.data[ yMap * _map.info.width + xMap ] > 70 ) {
	bumperMsg.contact = true;
    }
    bumperMsg.header.stamp = ros::Time::now();
    bumperMsg.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish(bumperMsg );
  }
}  // namespace stdr_robot
