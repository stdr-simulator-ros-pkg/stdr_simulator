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

#ifndef STDR_GUI_CONTROLLER
#define STDR_GUI_CONTROLLER

#include <iostream>
#include <cstdlib>

#include <boost/thread.hpp>

#include "stdr_gui/stdr_gui_connector.h"
#include "stdr_gui/stdr_info_connector.h"
#include "stdr_gui/stdr_map_connector.h"

#include "stdr_gui/stdr_visualization/stdr_sonar_visualization.h"
#include "stdr_gui/stdr_visualization/stdr_laser_visualization.h"

#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"

#include "nav_msgs/OccupancyGrid.h"

#include <stdr_robot/handle_robot.h>

namespace stdr_gui{

	/**
	 @class GuiController
	 @brief The main controller for the STDR GUI. Inherits QThread
	 **/ 
	class GuiController : public QThread{
		Q_OBJECT
		
		private: 
			int argc;
			char **argv;
			
			bool mapLock;
			
			std::map<std::string,GuiRobot> registeredRobots;	
			std::set<std::string> myRobots_;	
			stdr_msgs::RobotIndexedVectorMsg allRobots;	
			
			ros::Subscriber mapSubscriber;
			ros::Subscriber robotSubscriber;
			
			ros::NodeHandle n;
			
			nav_msgs::OccupancyGrid mapMsg;
			QImage initialMap;
			QImage runningMap;
			
			stdr_robot::HandleRobot robotHandler_;
			
			QTimer *timer;
			QTime elapsedTime;
			
			void fixRobotMsgAngles(stdr_msgs::RobotMsg& msg);
			
			std::map<QString,LaserVisualisation *> laserVisualizers;
			std::map<QString,SonarVisualisation *> sonarVisualizers;
			
		public:
			GuiController(int argc,char **argv);
			
			void setupWidgets(void);
		
			GuiConnector guiConnector;
			InfoConnector infoConnector;
			MapConnector mapConnector;
			
			void initializeCommunications(void);
			
			void receiveMap(const nav_msgs::OccupancyGrid& msg);
			void receiveRobots(const stdr_msgs::RobotIndexedVectorMsg& msg);
			 
			bool init();	
		
		public Q_SLOTS:
			void saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
			void loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
			void zoomInPressed(QPoint p);
			void zoomOutPressed(QPoint p);
			void robotPlaceSet(QPoint p);
			void updateMapInternal(void);
			void laserVisualizerClicked(QString robotName,QString laserName);
			void sonarVisualizerClicked(QString robotName,QString sonarName);
			void itemClicked(QPoint p);
			
		Q_SIGNALS:
			void waitForRobotPose(void);
			void updateMap(void);
	};
}

#endif

