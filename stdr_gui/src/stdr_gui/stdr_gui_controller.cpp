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
	
	void spinThreadFunction(void)
	{
		ros::spin();
	}
	
	CGuiController::CGuiController(int argc,char **argv):
		gui_connector_(argc,argv),
		info_connector_(argc,argv),
		map_connector_(argc,argv),
		argc_(argc),
		argv_(argv)
	{
		setupWidgets();
		
		map_lock_=false;
		
        icon_move_.addFile(QString::fromUtf8((
			getRosPackagePath("stdr_gui")+
			std::string("/resources/images/arrow_move.png")).c_str()), 
			QSize(20,20), QIcon::Normal, QIcon::Off);
        
        icon_delete_.addFile(QString::fromUtf8((
			getRosPackagePath("stdr_gui")+
			std::string("/resources/images/remove_icon.png")).c_str()), 
			QSize(20,20), QIcon::Normal, QIcon::Off);
	}
	
	CGuiController::~CGuiController(void)
	{
		
	}
	
	void CGuiController::initializeCommunications(void)
	{
		map_subscriber_=n_.subscribe(
			"map", 
			1, 
			&CGuiController::receiveMap,
			this);
			
		robot_subscriber_=n_.subscribe(
			"stdr_server/active_robots", 
			1, 
			&CGuiController::receiveRobots,
			this);
		
		QObject::connect(
			&gui_connector_,SIGNAL(setZoomInCursor(bool)),
			&map_connector_, SLOT(setCursorZoomIn(bool)));
		
		QObject::connect(
			&gui_connector_,SIGNAL(setZoomOutCursor(bool)),
			&map_connector_, SLOT(setCursorZoomOut(bool)));
		
		QObject::connect(
			&gui_connector_,SIGNAL(setAdjustedCursor(bool)),
			&map_connector_, SLOT(setCursorAdjusted(bool)));
		
		QObject::connect(
			&map_connector_,SIGNAL(zoomInPressed(QPoint)),
			this, SLOT(zoomInPressed(QPoint)));
		
		QObject::connect(
			&map_connector_,SIGNAL(zoomOutPressed(QPoint)),
			this, SLOT(zoomOutPressed(QPoint)));
		
		QObject::connect(
			&map_connector_,SIGNAL(itemClicked(QPoint,Qt::MouseButton)),
			this, SLOT(itemClicked(QPoint,Qt::MouseButton)));
		
		QObject::connect(
			&info_connector_,SIGNAL(laserVisualizerClicked(QString,QString)),
			this, SLOT(laserVisualizerClicked(QString,QString)));
		
		QObject::connect(
			&info_connector_,SIGNAL(sonarVisualizerClicked(QString,QString)),
			this, SLOT(sonarVisualizerClicked(QString,QString)));
		
		QObject::connect(
			&(gui_connector_.robotCreatorConn),
				SIGNAL(saveRobotPressed(stdr_msgs::RobotMsg)),
			this, SLOT(saveRobotPressed(stdr_msgs::RobotMsg)));
		
		QObject::connect(
			&(gui_connector_.robotCreatorConn),
				SIGNAL(loadRobotPressed(stdr_msgs::RobotMsg)),
			this, SLOT(loadRobotPressed(stdr_msgs::RobotMsg)));
		
		QObject::connect(
			this,SIGNAL(waitForRobotPose()),
			&map_connector_, SLOT(waitForPlace()));
		
		QObject::connect(
			&map_connector_,SIGNAL(robotPlaceSet(QPoint)),
			this, SLOT(robotPlaceSet(QPoint)));
		
		QObject::connect(
			this,SIGNAL(updateMap()),
			this, SLOT(updateMapInternal()));
		
		timer_=new QTimer(this);
		connect(
			timer_, SIGNAL(timeout()), 
			this, SLOT(updateMapInternal()));
	}
	
	void CGuiController::setupWidgets(void)
	{
		{
			gui_connector_.addToGrid(info_connector_.getLoader(),0,0);
		}
		{
			initial_map_=running_map_=QImage((
				getRosPackagePath("stdr_gui")+
				std::string("/resources/images/logo.png")).c_str());

			map_msg_.info.width=initial_map_.width();
			map_msg_.info.height=initial_map_.height();
			
			map_connector_.updateImage(&running_map_);
			
			gui_connector_.addToGrid(map_connector_.getLoader(),0,1);

			gui_connector_.setGridColumnStretch(1,5);
			gui_connector_.setGridColumnStretch(0,2);
		}
	}
	
	bool CGuiController::init(void)
	{
		if ( ! ros::master::check() ) {
			return false;
		}
		gui_connector_.show();

		initializeCommunications();
		boost::thread spinThread(&spinThreadFunction);
		return true;
	}

	void CGuiController::receiveMap(const nav_msgs::OccupancyGrid& msg)
	{
		map_msg_=msg;
		initial_map_=running_map_=
			QImage(msg.info.width,msg.info.height,QImage::Format_RGB32);
		QPainter painter(&running_map_);
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
		
		initial_map_=running_map_;

		gui_connector_.setMapLoaded(true);
		info_connector_.updateMapInfo(msg.info.width*msg.info.resolution,
									msg.info.height*msg.info.resolution,
									msg.info.resolution);
		map_connector_.setInitialImageSize(
			QSize(initial_map_.width(),initial_map_.height()));
		
		elapsed_time_.start();
		
		timer_->start(200);
	}
	
	void CGuiController::saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg)
	{
		ROS_ERROR("Save Signal ok");
	}
	void CGuiController::loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg)
	{
		Q_EMIT waitForRobotPose();
	}
	
	void CGuiController::zoomInPressed(QPoint p)
	{
		map_connector_.updateZoom(p,true);
	}
	void CGuiController::zoomOutPressed(QPoint p)
	{
		map_connector_.updateZoom(p,false);
	}
	
	void CGuiController::receiveRobots(
		const stdr_msgs::RobotIndexedVectorMsg& msg)
	{
		while(map_lock_)	
			usleep(100);
		map_lock_=true;
		registered_robots_.clear();
		all_robots_=msg;
		for(unsigned int i=0;i<msg.robots.size();i++){
			registered_robots_.push_back(CGuiRobot(msg.robots[i]));
		}
		info_connector_.updateTree(msg);
		map_lock_=false;
	}
	
	void CGuiController::robotPlaceSet(QPoint p)
	{
		while(map_lock_)	
			usleep(100);
		map_lock_=true;
		QPoint pnew=map_connector_.getGlobalPoint(p);
		
		gui_connector_.robotCreatorConn.setInitialPose(QPoint(
			pnew.x()*map_msg_.info.resolution,
			pnew.y()*map_msg_.info.resolution));
			
		stdr_msgs::RobotIndexedMsg newRobot;
		gui_connector_.robotCreatorConn.fixRobotMsgAngles();
		try {
			newRobot=robot_handler_.spawnNewRobot(
				gui_connector_.robotCreatorConn.getNewRobot());
		}
		catch (ConnectionException& ex) {
			ROS_ERROR("%s", ex.what());
			return;
		}
		my_robots_.insert(newRobot.name);
		map_lock_=false;
	}
	
	void CGuiController::updateMapInternal(void)
	{
		while(map_lock_)
			usleep(100);
		map_lock_=true;
		running_map_=initial_map_;
		
		if(gui_connector_.isGridEnabled())
			map_connector_.drawGrid(&(running_map_),map_msg_.info.resolution);
		
		for(unsigned int i=0;i<registered_robots_.size();i++){
			registered_robots_[i].draw(
				&running_map_,map_msg_.info.resolution,&listener_);
		}
		running_map_=running_map_.mirrored(false,true);
		for(unsigned int i=0;i<registered_robots_.size();i++){
			if(registered_robots_[i].getShowLabel())
				registered_robots_[i].drawLabel(
					&running_map_,map_msg_.info.resolution);
		}

		map_connector_.updateImage(&(running_map_));
		
		gui_connector_.setStatusBarMessage(
			QString("Time elapsed : ")+
			getLiteralTime(elapsed_time_.elapsed()));
		map_lock_=false;
		
		//Check if all visualisers are active
		std::vector<QString> toBeErased;
		for(LaserVisIterator it=laser_visualizers_.begin();
			it!=laser_visualizers_.end();
			it++)
		{
			if(!it->second->getActive()){
				toBeErased.push_back(it->first);
			}
			else{
				it->second->paint();
			}
		}
		for(unsigned int i=0;i<toBeErased.size();i++){
			laser_visualizers_.erase(toBeErased[i]);
		}
		toBeErased.clear();
		for(SonarVisIterator it=sonar_visualizers_.begin();
			it!=sonar_visualizers_.end();
			it++)
		{
			if(!it->second->getActive()){
				toBeErased.push_back(it->first);
			}
		}
		for(unsigned int i=0;i<toBeErased.size();i++){
			sonar_visualizers_.erase(toBeErased[i]);
		}
	}
	
	stdr_msgs::LaserSensorMsg CGuiController::getLaserDescription(
		QString robotName,
		QString laserName)
	{
		for(unsigned int i=0;i<all_robots_.robots.size();i++){					
			if(all_robots_.robots[i].name==robotName.toStdString()){
				for(unsigned int j=0;
					j<all_robots_.robots[i].robot.laserSensors.size();
					j++)
				{
					if(all_robots_.robots[i].robot.laserSensors[j].frame_id
							==laserName.toStdString())
					{
						return all_robots_.robots[i].robot.laserSensors[j];
					}
				}
			}
		}
		return stdr_msgs::LaserSensorMsg();			
	}
				
	stdr_msgs::SonarSensorMsg CGuiController::getSonarDescription(
		QString robotName,
		QString sonarName)
	{
		for(unsigned int i=0;i<all_robots_.robots.size();i++){					
			if(all_robots_.robots[i].name==robotName.toStdString()){
				for(unsigned int j=0;
					j<all_robots_.robots[i].robot.sonarSensors.size();
					j++)
				{
					if(all_robots_.robots[i].robot.sonarSensors[j].frame_id
							==sonarName.toStdString())
					{
						return all_robots_.robots[i].robot.sonarSensors[j];
					}
				}
			}
		}	
		return stdr_msgs::SonarSensorMsg();						
	}
	
	void CGuiController::laserVisualizerClicked(
		QString robotName,
		QString laserName)
	{
		QString name=robotName+QString("/")+laserName;
		if(laser_visualizers_.find(name)!=laser_visualizers_.end())
			return;
		CLaserVisualisation *lv;
		lv=new CLaserVisualisation(name,map_msg_.info.resolution);
		laser_visualizers_.insert(
			std::pair<QString,CLaserVisualisation *>(name,lv));
		lv->setWindowFlags(Qt::WindowStaysOnTopHint);

		lv->setLaser(getLaserDescription(robotName,laserName));
		
		lv->show();
	}
	void CGuiController::sonarVisualizerClicked(
		QString robotName,
		QString sonarName)
	{
		QString name=robotName+QString("/")+sonarName;
		if(sonar_visualizers_.find(name)!=sonar_visualizers_.end())
			return;
		CSonarVisualisation *sv;
		sv=new CSonarVisualisation(name);
		sonar_visualizers_.insert(
			std::pair<QString,CSonarVisualisation *>(name,sv));
		sv->setWindowFlags(Qt::WindowStaysOnTopHint);

		sv->setSonar(getSonarDescription(robotName,sonarName));
						
		sv->show();
	}
	
	void CGuiController::itemClicked(QPoint p,Qt::MouseButton b)
	{
		QPoint pointClicked=map_connector_.getGlobalPoint(p);
		for(unsigned int i=0;i<registered_robots_.size();i++){
			if(registered_robots_[i].checkEventProximity(pointClicked)){
				if(b==Qt::RightButton){
					QMenu myMenu;
					QAction *deleteRobot=
						myMenu.addAction(icon_delete_,"Delete robot");
					QAction *moveRobot=
						myMenu.addAction(icon_move_,"Move robot");
					myMenu.addSeparator();
					QAction *showCircle=
						myMenu.addAction("Show proximity circles");
					QAction* selectedItem = 
						myMenu.exec(map_connector_.mapToGlobal(p));
					if(selectedItem==showCircle){
						registered_robots_[i].toggleShowCircles();
					}
				}
				else if(b==Qt::LeftButton){
					registered_robots_[i].toggleShowLabel();
				}
			}
		}
	}
}	


