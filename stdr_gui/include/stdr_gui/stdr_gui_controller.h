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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"

#include "nav_msgs/OccupancyGrid.h"

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
			
			std::vector<GuiRobot> registeredRobots;			
			
			ros::Subscriber mapSubscriber;
			
			ros::NodeHandle n;
			
			nav_msgs::OccupancyGrid mapMsg;
			QImage initialMap;
			QImage runningMap;
			
			
		public:
			GuiController(int argc,char **argv);
			
			void setupWidgets(void);
		
			GuiConnector guiConnector;
			InfoConnector infoConnector;
			MapConnector mapConnector;
			
			void initializeCommunications(void);
			
			void receiveMap(const nav_msgs::OccupancyGrid& msg);
			 
			bool init();	
	};
}

#endif

