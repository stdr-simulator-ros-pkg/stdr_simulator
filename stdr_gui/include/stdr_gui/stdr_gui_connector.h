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

namespace stdr_gui
{
	
	/**
	 @class GuiConnector
	 @brief Serves the Qt events of the main GUI window. Inherits from QObject
	 **/ 
	class CGuiConnector:
		public QObject
	{
		Q_OBJECT
		
		private:
		
			int 	argc_; 			//!< Number of input arguments
			char**	argv_;			//!< Input arguments
			bool 	map_loaded_;	//!< True if any map is loaded from server
			
			bool 	grid_enabled_;
			
			CGuiLoader loader_;		//!< The loader of main GUI QWidget 
			
		public:
		
									
			CRobotCreatorConnector robotCreatorConn;		//!< Serves the Qt events of the RobotCreator Widget
			
			/**
			@brief Deafault contructor
			@param argc [int] Number of input arguments
			@param argv [char **] Input arguments
			@return void
			**/
			CGuiConnector(int argc, char **argv);
			
			/**
			@brief Setter of _mapLoaded variable
			@param mapLoaded [bool] 
			@return void
			**/
			void setMapLoaded(bool mapLoaded);
			
			bool isGridEnabled(void);
			
			void addToGrid(QWidget *w,int row,int column);
			
			void setGridColumnStretch(int cell,int stretch);
			
			void show(void);
			
			void setStatusBarMessage(QString s);
			
			QEvent* getCloseEvent(void);
			
			bool closeTriggered(void);
			
			void shutdown(void);

		public Q_SLOTS:
		
			/**
			@brief Qt slot that is called when the Properties tool button is pressed
			@return void
			**/
			void actionPropertiesTriggered(void);
			
			/**
			@brief Qt slot that is called when the About tool button is pressed
			@return void
			**/
			void actionAboutTriggered(void);
			
			/**
			@brief Qt slot that is called when the Exit action is triggered
			@return void
			**/
			void actionExitTriggered(void);
			
			/**
			@brief Qt slot that is called when the LoadMap tool button is pressed
			@return void
			**/
			void actionLoadMapTriggered(void);
			
			/**
			@brief Qt slot that is called when the NewRobot tool button is pressed
			@return void
			**/
			void actionNewRobotTriggered(void);
			
			void actionNewRfidTriggered(void);
			void actionNewCo2Triggered(void);
			void actionNewThermalTriggered(void);
			
			/**
			@brief Qt slot that is called when the zoom in tool button is pressed
			@return void
			**/
			void actionZoomInTriggered(void);
			
			/**
			@brief Qt slot that is called when the zoom out tool button is pressed
			@return void
			**/
			void actionZoomOutTriggered(void);
			
			void actionAdjustedTriggered(void); 
			void actionGridTriggered(void);
			
		Q_SIGNALS:
		
			/**
			@brief Qt signal that is emmited in GuiConnector::actionZoomInTriggered and connects to MapLoader::setCursorZoomIn
			@param state [bool] Toggle flag
			@return void
			**/
			void setZoomInCursor(bool state);
			
			/**
			@brief Qt signal that is emmited in GuiConnector::actionZoomOutTriggered and connects to MapLoader::setCursorZoomOut
			@param state [bool] Toggle flag
			@return void
			**/
			void setZoomOutCursor(bool state);
			void setAdjustedCursor(bool state);
			
			void guiExitEvent(void);
			
			void loadRfidPressed(void);
			void loadThermalPressed(void);
			void loadCo2Pressed(void);
			
			
	};
}

#endif
