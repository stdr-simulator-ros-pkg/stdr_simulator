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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_laser.h"

namespace stdr_gui{
	CGuiLaser::CGuiLaser(stdr_msgs::LaserSensorMsg msg,std::string baseTopic):
		msg_(msg)
	{
		topic_=baseTopic+"/"+msg_.frame_id;
		ros::NodeHandle n;
		lock_=false;
		subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiLaser::callback,this);
	}
	
	CGuiLaser::~CGuiLaser(void)
	{
		
	}
	
	void CGuiLaser::callback(const sensor_msgs::LaserScan& msg)
	{
		if(lock_){
			return;
		}
		scan_=msg;
	}
	
	void CGuiLaser::paint(
		QImage *m,
		float ocgd,
		geometry_msgs::Pose2D robotPose)
	{
		lock_=true;
		QPainter painter(m);
		painter.setPen(QColor(255,0,0,100));
		for(unsigned int i=0;i<scan_.ranges.size();i++){
			painter.drawLine(
				robotPose.x/ocgd+msg_.pose.x/ocgd,
				robotPose.y/ocgd+msg_.pose.y/ocgd,
				robotPose.x/ocgd+msg_.pose.x/ocgd+
					scan_.ranges[i]*
					cos(robotPose.theta+scan_.angle_min+i*scan_.angle_increment)
					/ocgd,
				robotPose.y/ocgd+msg_.pose.y/ocgd+
					scan_.ranges[i]*
					sin(robotPose.theta+scan_.angle_min+i*scan_.angle_increment)
					/ocgd
			);				
		}
		lock_=false;
	}
}

