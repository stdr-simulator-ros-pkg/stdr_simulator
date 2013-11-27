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

namespace stdr_robot {

class Laser : public Sensor {
	
	public:
	
		Laser(const nav_msgs::OccupancyGridConstPtr& map, 
				tf::TransformBroadcaster& tf,
				const stdr_msgs::LaserSensorMsg& msg, 
				const std::string& name, 
				ros::NodeHandle& n);
		void callback(const ros::TimerEvent&);
		void tfCallback(const ros::TimerEvent&);
		
		~Laser() {}
		
	private:
		
		stdr_msgs::LaserSensorMsg _description;
	
};
	
}

#endif
