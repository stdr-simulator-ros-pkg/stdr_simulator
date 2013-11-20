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

#ifndef STDR_GUI_CONNECTOR
#define STDR_GUI_CONNECTOR

#include "stdr_gui/stdr_gui_loader.h"
#include "stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.h"

namespace stdr{
	class GuiConnector:public QObject{
		Q_OBJECT
		
			int argc; 
			char **argv;
			
		public:
			GuiConnector(int argc, char **argv);
			GuiLoader loader;
			RobotCreatorConnector robotCreatorConn;

		public Q_SLOTS:
			void actionPropertiesTriggered(void);
			void actionAboutTriggered(void);
			void actionExitTriggered(void);
			void actionLoadMapTriggered(void);
			void actionNewRobotTriggered(void);
	};
}

#endif
