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
	GuiLaser::GuiLaser(stdr_msgs::LaserSensorMsg msg,std::string baseTopic):
		_msg(msg)
	{
		_topic=baseTopic+"/"+_msg.frame_id;
		ros::NodeHandle _n;
		_lock=false;
		_subscriber = _n.subscribe(_topic.c_str(), 1, &GuiLaser::callback,this);
	}
	
	void GuiLaser::callback(const sensor_msgs::LaserScan& msg){
		if(_lock){
			return;
		}
		_scan=msg;
	}
	
	void GuiLaser::paint(QImage *m,float ocgd,geometry_msgs::Pose2D robotPose){	//block spinning?
		_lock=true;
		QPainter painter(m);
		painter.setPen(QColor(255,0,0,100));
		for(unsigned int i=0;i<_scan.ranges.size();i++){
			painter.drawLine(
				robotPose.x/ocgd+_msg.pose.x/ocgd,
				robotPose.y/ocgd+_msg.pose.y/ocgd,
				robotPose.x/ocgd+_msg.pose.x/ocgd+_scan.ranges[i]*cos(robotPose.theta+_scan.angle_min+i*_scan.angle_increment)/ocgd,
				robotPose.y/ocgd+_msg.pose.y/ocgd+_scan.ranges[i]*sin(robotPose.theta+_scan.angle_min+i*_scan.angle_increment)/ocgd
			);				
		}
		_lock=false;
	}
}

