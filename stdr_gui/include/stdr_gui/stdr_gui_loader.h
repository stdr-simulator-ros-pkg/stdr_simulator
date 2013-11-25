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

#ifndef STDR_GUI_LOADER
#define STDR_GUI_LOADER

#include "ui_simulator.h"
#include "stdr_tools.h"

namespace stdr{
	
	class GuiLoader : public Ui_MainWindow, public QMainWindow{
			
			int argc;
			char **argv;

		public:
			QAction *actionProperties;
			QAction *actionGrid;
			QAction *actionNewRobot;
			QAction *actionNewRfid;
			QAction *actionLoadMap;
			QAction *actionZoomIn;
			QAction *actionZoomOut;
		
			GuiLoader(int argc,char **argv);
			
			void addToolbarIcons(void);
	};
	
}

#endif
