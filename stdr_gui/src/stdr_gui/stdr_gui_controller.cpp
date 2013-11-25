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

#include "stdr_gui/stdr_gui_controller.h"


namespace stdr_gui{
	
	void spinThreadFunction(void){
		ros::spin();
	}
	
	GuiController::GuiController(int argc,char **argv):
			guiConnector(argc,argv),
			infoConnector(argc,argv),
			mapConnector(argc,argv)
	{
		this->argc=argc;
		this->argv=argv;
		setupWidgets();
		
		mapLock=false;
	}
	
	void GuiController::initializeCommunications(void){
		mapSubscriber=n.subscribe("map", 1, &GuiController::receiveMap,this);
		robotSubscriber=n.subscribe("stdr_server/active_robots", 1, &GuiController::receiveRobots,this);
		
		QObject::connect(&guiConnector,SIGNAL(setZoomInCursor(bool)),&mapConnector, SLOT(setCursorZoomIn(bool)));
		QObject::connect(&guiConnector,SIGNAL(setZoomOutCursor(bool)),&mapConnector, SLOT(setCursorZoomOut(bool)));
		QObject::connect(&mapConnector,SIGNAL(zoomInPressed(QPoint)),this, SLOT(zoomInPressed(QPoint)));
		QObject::connect(&mapConnector,SIGNAL(zoomOutPressed(QPoint)),this, SLOT(zoomOutPressed(QPoint)));
		
		QObject::connect(&(guiConnector.robotCreatorConn),SIGNAL(saveRobotPressed(stdr_msgs::RobotMsg)),this, SLOT(saveRobotPressed(stdr_msgs::RobotMsg)));
		QObject::connect(&(guiConnector.robotCreatorConn),SIGNAL(loadRobotPressed(stdr_msgs::RobotMsg)),this, SLOT(loadRobotPressed(stdr_msgs::RobotMsg)));
		QObject::connect(this,SIGNAL(waitForRobotPose()),&mapConnector, SLOT(waitForPlace()));
		QObject::connect(&mapConnector,SIGNAL(robotPlaceSet(QPoint)),this, SLOT(robotPlaceSet(QPoint)));
		QObject::connect(this,SIGNAL(updateMap()),this, SLOT(updateMapInternal()));
		
		timer=new QTimer(this);
		connect(timer, SIGNAL(timeout()), this, SLOT(updateMapInternal()));
		
	}
	
	void GuiController::setupWidgets(void){
		{
			guiConnector.loader.gridLayout->addWidget(static_cast<QWidget *>(&infoConnector.loader),0,0,0);	
		}
		{
			initialMap=runningMap=QImage((getRosPackagePath("stdr_gui")+std::string("/resources/images/logo.png")).c_str());

			mapMsg.info.width=initialMap.width();
			mapMsg.info.height=initialMap.height();
			
			mapConnector.updateImage(&runningMap);
			
			guiConnector.loader.gridLayout->addWidget(static_cast<QWidget *>(&mapConnector.loader),0,1,0);	
			guiConnector.loader.gridLayout->setColumnStretch(1,1);
		}
	}
	
	bool GuiController::init(void){
		if ( ! ros::master::check() ) {
			return false;
		}
		guiConnector.loader.show();

		initializeCommunications();
		boost::thread spinThread(&spinThreadFunction);
		return true;
	}

	void GuiController::receiveMap(const nav_msgs::OccupancyGrid& msg){
		mapMsg=msg;
		initialMap=runningMap=QImage(msg.info.width,msg.info.height,QImage::Format_RGB32);
		QPainter painter(&runningMap);
		int d=0;
		QColor c;
		for(unsigned int i=0;i<msg.info.width;i++){
			for(unsigned int j=0;j<msg.info.height;j++){
				if(msg.data[j*msg.info.width+i]==-1)
					c=QColor(127,127,127);
				else{
					d=(100.0-msg.data[j*msg.info.width+i])/100.0*255.0;
					c=QColor(d,d,d);
				}
				painter.setPen(c);
				painter.drawPoint(i,j);
			}
		}	
		int originx=msg.info.origin.position.x;
		int originy=msg.info.origin.position.y;
		painter.setPen(Qt::blue);
		painter.drawLine(originx,originy-20,originx,originy+20);
		painter.drawLine(originx-20,originy,originx+20,originy);
		
		initialMap=runningMap;
		Q_EMIT updateMap();

		guiConnector.setMapLoaded(true);
		timer->start(200);
	}
	
	void GuiController::saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg){
		ROS_ERROR("Save Signal ok");
	}
	void GuiController::loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg){
		Q_EMIT waitForRobotPose();
	}
	
	void GuiController::zoomInPressed(QPoint p){
		ROS_ERROR("zoomInPressed Signal ok");
	}
	void GuiController::zoomOutPressed(QPoint p){
		ROS_ERROR("zoomOutPressed Signal ok");
	}
	
	void GuiController::receiveRobots(const stdr_msgs::RobotIndexedVectorMsg& msg){
		while(mapLock)	usleep(100);
		mapLock=true;
		registeredRobots.clear();
		
		for(unsigned int i=0;i<msg.robots.size();i++){
			stdr_msgs::RobotIndexedMsg m=msg.robots[i];
			registeredRobots.insert(std::pair<std::string,GuiRobot>(msg.robots[i].name,GuiRobot(m)));
		}
		mapLock=false;
	}
	
	void GuiController::robotPlaceSet(QPoint p){
		while(mapLock)	usleep(100);
		mapLock=true;
		QPoint pnew=pointFromImage(p);
		guiConnector.robotCreatorConn.newRobotMsg.initialPose.x=pnew.x()*mapMsg.info.resolution;
		guiConnector.robotCreatorConn.newRobotMsg.initialPose.y=pnew.y()*mapMsg.info.resolution;
		stdr_msgs::RobotIndexedMsg newRobot;
		try {
			newRobot=robotHandler_.spawnNewRobot(guiConnector.robotCreatorConn.newRobotMsg);
		}
		catch (ConnectionException& ex) {
			ROS_ERROR("%s", ex.what());
			return;
		}
		myRobots_.insert(newRobot.name);
		mapLock=false;
	}
	
	void GuiController::updateMapInternal(void){
		while(mapLock)	usleep(100);
		mapLock=true;
		runningMap=initialMap;
		for(std::map<std::string,GuiRobot>::iterator it=registeredRobots.begin();it!=registeredRobots.end();it++){
			it->second.draw(&runningMap,mapMsg.info.resolution);
		}
		runningMap=runningMap.mirrored(false,true);
		for(std::map<std::string,GuiRobot>::iterator it=registeredRobots.begin();it!=registeredRobots.end();it++){
			it->second.drawLabel(&runningMap,mapMsg.info.resolution);
		}
		mapConnector.loader.updateImage(&(runningMap));
		mapLock=false;
	}
	
	QPoint GuiController::pointFromImage(QPoint p){
		QPoint newPoint;
		float x=p.x();
		float y=p.y();
		float initialWidth=initialMap.width();
		float currentWidth=mapConnector.loader.map->width();
		float climax=initialWidth/currentWidth;
		newPoint.setX(x*climax);
		newPoint.setY(initialMap.height()-y*climax);
		return newPoint;
	}
}

