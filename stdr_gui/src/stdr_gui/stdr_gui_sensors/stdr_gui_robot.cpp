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
		robotInitialized=false;
		initialPose=msg.robot.initialPose;
		currentPose=initialPose;
		footprint=msg.robot.footprint;
		radius=msg.robot.footprint.radius;
		frameId_=msg.name;
		showLabel=true;
		showCircles=false;
		for(unsigned int i=0;i<msg.robot.laserSensors.size();i++){
			GuiLaser *l=new GuiLaser(msg.robot.laserSensors[i],frameId_);
			_lasers.push_back(l);
		}
		robotInitialized=true;
	}

	void GuiRobot::draw(QImage *m,float ocgd,tf::TransformListener *_listener){
		if(!robotInitialized) return;
		resolution=ocgd;
		tf::StampedTransform transform;
		
		try{
			_listener->lookupTransform("map", frameId_.c_str(),ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_DEBUG("%s",ex.what());
		}
		tfScalar roll,pitch,yaw;
		currentPose.x=transform.getOrigin().x();
		currentPose.y=transform.getOrigin().y();
		transform.getBasis().getRPY(roll,pitch,yaw);
		currentPose.theta=yaw;
		drawSelf(m);
		for(unsigned int i=0;i<_lasers.size();i++){
			_lasers[i]->paint(m,ocgd,currentPose);
		}
	}
	void GuiRobot::drawSelf(QImage *m){
		QPainter painter(m);
		painter.setPen(Qt::blue);
		painter.drawEllipse((currentPose.x-radius/2)/resolution,(currentPose.y-radius/2)/resolution,radius/resolution,radius/resolution);
		painter.drawLine(	currentPose.x/resolution,
							currentPose.y/resolution,
							currentPose.x/resolution+radius/resolution*1.05*cos(currentPose.theta),
							currentPose.y/resolution+radius/resolution*1.05*sin(currentPose.theta));
		
		if(showCircles){
			painter.setPen(QColor(255,0,0,150));
			painter.drawEllipse((currentPose.x-1.0/2)/resolution,(currentPose.y-1.0/2)/resolution,1.0/resolution,1.0/resolution);
			painter.drawEllipse((currentPose.x-2.0/2)/resolution,(currentPose.y-2.0/2)/resolution,2.0/resolution,2.0/resolution);
			painter.drawEllipse((currentPose.x-3.0/2)/resolution,(currentPose.y-3.0/2)/resolution,3.0/resolution,3.0/resolution);
			painter.drawEllipse((currentPose.x-4.0/2)/resolution,(currentPose.y-4.0/2)/resolution,4.0/resolution,4.0/resolution);
			painter.drawEllipse((currentPose.x-5.0/2)/resolution,(currentPose.y-5.0/2)/resolution,5.0/resolution,5.0/resolution);
		}
		
	}
	
	bool GuiRobot::checkEventProximity(QPoint p){
		float dx=p.x()*resolution-currentPose.x;
		float dy=p.y()*resolution-currentPose.y;
		float dist=sqrt(pow(dx,2)+pow(dy,2));
		return dist<=radius;
	}
	
	GuiRobot::~GuiRobot(void){}
	
	std::string GuiRobot::getFrameId(void){
		return frameId_;
	}
	
	void GuiRobot::drawLabel(QImage *m,float ocgd){
		QPainter painter(m);
		painter.setPen(Qt::black);
		painter.drawRect(currentPose.x/ocgd+10,m->height()-(currentPose.y/ocgd)-30,100,20);
		painter.setPen(Qt::white);
		painter.fillRect(currentPose.x/ocgd+10,m->height()-(currentPose.y/ocgd)-30,100,20,QBrush(QColor(0,0,0,140)));
		painter.drawText(currentPose.x/ocgd+12,m->height()-(currentPose.y/ocgd)-15,QString(frameId_.c_str()));
	}
	
	void GuiRobot::setShowLabel(bool b){
		showLabel=b;
	}
	
	bool GuiRobot::getShowLabel(void){
		return showLabel;
	}
	
	void GuiRobot::toggleShowLabel(void){
		showLabel=!showLabel;
	}
	
	void GuiRobot::toggleShowCircles(void){
		showCircles=!showCircles;
	}
}
