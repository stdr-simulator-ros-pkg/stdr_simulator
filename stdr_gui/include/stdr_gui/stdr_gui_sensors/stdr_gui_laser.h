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

#ifndef STDR_GUI_LASER_CONTAINER
#define STDR_GUI_LASER_CONTAINER

#include "stdr_gui/stdr_tools.h"
#include "stdr_msgs/LaserSensorMsg.h"
#include "sensor_msgs/LaserScan.h"

namespace stdr{
	class GuiLaser{
			std::string _topic;
			const stdr_msgs::LaserSensorMsg& _msg;
			ros::Subscriber _subscriber;
			QImage _mapImage;

		public:
			GuiLaser(stdr_msgs::LaserSensorMsg msg,QImage mapImage,std::string baseTopic);
			~GuiLaser(void){}
			void callback(const sensor_msgs::LaserScan& msg); 
			void paint(void);
	};	
}

#endif
