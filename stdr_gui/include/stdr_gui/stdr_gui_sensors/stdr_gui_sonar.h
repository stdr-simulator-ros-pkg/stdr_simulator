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


#ifndef STDR_GUI_SONAR_CONTAINER
#define STDR_GUI_SONAR_CONTAINER

#include "stdr_gui/stdr_tools.h"
#include "stdr_msgs/SonarSensorMsg.h"
#include "sensor_msgs/Range.h"

namespace stdr_gui
{

	class CGuiSonar
	{
		private:
			bool lock_;
			
			std::string topic_;
			
			stdr_msgs::SonarSensorMsg msg_;
			
			ros::Subscriber subscriber_;
			sensor_msgs::Range range_;
		
		public:
			CGuiSonar(stdr_msgs::SonarSensorMsg msg,std::string baseTopic);
			~CGuiSonar(void);
			
			void callback(const sensor_msgs::Range& msg); 
			void paint(QImage *m,float ocgd,geometry_msgs::Pose2D robotPose);
			void visualizerPaint(QImage *m,float ocgd,float maxRange);
			float getMaxRange(void);
	};	
}

#endif
