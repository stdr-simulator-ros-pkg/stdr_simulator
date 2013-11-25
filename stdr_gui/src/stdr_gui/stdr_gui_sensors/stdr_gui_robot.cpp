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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"

namespace stdr_gui{
	GuiRobot::GuiRobot(const stdr_msgs::RobotIndexedMsg& msg){
		initialPose=msg.robot.initialPose;
		currentPose=initialPose;
		footprint=msg.robot.footprint;
		radius=msg.robot.radius;
		frameId_=msg.name;
		// Setup rest of sensors
	}
	
	GuiRobot::GuiRobot(void){}
	
	GuiRobot::GuiRobot(const GuiRobot& other){
		this->_lasers=other._lasers;
		this->_sonars=other._sonars;
		this->_rfids=other._rfids;
		this->initialPose=other.initialPose;
		this->currentPose=other.currentPose;
		this->footprint=other.footprint;
		this->radius=other.radius;
		this->frameId_=other.frameId_;
	}
	
	void GuiRobot::draw(QImage *m,float ocgd){
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("map", frameId_.c_str(),ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		currentPose.x=transform.getOrigin().x();
		currentPose.y=transform.getOrigin().y();
		drawSelf(m,ocgd);
		// Call draw for sensors
	}
	void GuiRobot::drawSelf(QImage *m,float ocgd){
		QPainter painter(m);
		painter.setPen(Qt::blue);
		painter.drawEllipse((currentPose.x-radius/2)/ocgd,(currentPose.y-radius/2)/ocgd,radius/ocgd,radius/ocgd);
		painter.drawLine(	currentPose.x/ocgd,
							currentPose.y/ocgd,
							currentPose.x/ocgd+radius/ocgd*1.05*cos(currentPose.theta),
							currentPose.y/ocgd+radius/ocgd*1.05*sin(currentPose.theta));
	}
	
	GuiRobot::~GuiRobot(void){}
}
