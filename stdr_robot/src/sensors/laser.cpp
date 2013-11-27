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

#include <stdr_robot/sensors/laser.h>

namespace stdr_robot {
	
Laser::Laser(const nav_msgs::OccupancyGridConstPtr& map, 
	tf::TransformBroadcaster& tf, 
	const stdr_msgs::LaserSensorMsg& msg, 
	const std::string& name,
	ros::NodeHandle& n)
  : Sensor(map, tf, name)
{
	_description = msg;
	_timer = n.createTimer(ros::Duration(1/msg.frequency), &Laser::callback, this);	
}

void Laser::callback(const ros::TimerEvent&) {
	// calculate laser
}

void Laser::tfCallback(const ros::TimerEvent&) {
	// publish laser tf
}
	
}
