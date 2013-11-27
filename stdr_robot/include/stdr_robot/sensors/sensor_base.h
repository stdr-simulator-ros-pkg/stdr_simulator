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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

namespace stdr_robot {

class Sensor {
	
	public:
		
		virtual void callback(const ros::TimerEvent&) = 0;
		virtual void tfCallback(const ros::TimerEvent&) = 0;
		virtual ~Sensor() {}
	
	protected:
		
		Sensor(const nav_msgs::OccupancyGridConstPtr& map, 
				tf::TransformBroadcaster& tf, 
				const std::string& name) 
		: _map(map), _broadcaster(tf), _namespace(name) {}
		
	protected:
	
		const std::string& _namespace;
		const nav_msgs::OccupancyGridConstPtr& _map;
		ros::Timer _timer;
		ros::Timer _tfTimer;
		tf::TransformBroadcaster& _broadcaster;
		ros::Publisher _publisher;
};

typedef boost::shared_ptr<Sensor> SensorPtr;
typedef std::vector<SensorPtr> SensorPtrVector;
	
}

#endif
