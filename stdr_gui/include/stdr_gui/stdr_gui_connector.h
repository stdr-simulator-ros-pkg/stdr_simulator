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

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  
  /**
  @class CGuiConnector
  @brief Serves the Qt events of the main GUI window. Inherits from QObject
  **/ 
  class CGuiConnector:
    public QObject
  {
    Q_OBJECT
    
    //------------------------------------------------------------------------//
    private:
    
      //!< Number of input arguments
      int   argc_;  
      //!< Input arguments     
      char**  argv_;   
      //!< True if any map is loaded from server
      bool map_initialized_;
      //!< True if grid is enabled
      bool   grid_enabled_;
      
      //!< The loader of main GUI QWidget 
      CGuiLoader loader_;    
    
    //------------------------------------------------------------------------//
    public:
      
      //!< Serves the Qt events of the RobotCreator Widget
      CRobotCreatorConnector robotCreatorConn;    
      
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CGuiConnector(int argc, char **argv);
      
      /**
      @brief Returns the grid enabled state
      @return bool : True if grid is enabled
      **/
      bool isGridEnabled(void);
      
      /**
      @brief Raises a message box with a specific message
      @param title [QString] The message box title
      @param s [QString] The message
      @return void
      **/
      void raiseMessage(QString title, QString s);
      
      /**
      @brief Adds a widget to the main window Qt grid
      @param w [QWidget*] The widget to be placed
      @param row [int] The row of the grid
      @param column [int] The column of the grid
      @return void
      **/
      void addToGrid(QWidget *w,int row,int column);
      
      /**
      @brief Wraps the Qt gridColumnStretch function
      @param cell [int] The specific column
      @param stretch [int] The relative stretch coefficient
      @return void
      **/
      void setGridColumnStretch(int cell,int stretch);
      
      /**
      @brief Shows the main window
      @return void
      **/
      void show(void);
      
      /**
      @brief Displays a message in the QMainWindow status bar
      @param s [QString] The message
      @return void
      **/
      void setStatusBarMessage(QString s);
      
      /**
      @brief Returns the exit event captured
      @return QEvent* The captured event
      **/
      QEvent* getCloseEvent(void);
      
      /**
      @brief Returns the exit triggered status
      @return bool True if exit has been triggered
      **/
      bool closeTriggered(void);
      
      /**
      @brief Shuts down the main window
      @return void
      **/
      void shutdown(void);
      
      /**
      @brief Sets the map_initialized_ private variable
      @param mi [bool] The new value
      @return void
      **/
      void setMapInitialized(bool mi);
      
      /**
      @brief Unchecks the zoom in & out buttons when right click in map is pushed
      @return void
      **/
      void uncheckZoomButtons(void);
    
    //------------------------------------------------------------------------//
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
      
      /**
      @brief Qt slot that is called when the NewRfid tool button is pressed
      @return void
      **/
      void actionNewRfidTriggered(void);
      
      /**
      @brief Qt slot that is called when the NewCO2 tool button is pressed
      @return void
      **/
      void actionNewCo2Triggered(void);
      
      /**
      @brief Qt slot that is called when the NewSound tool button is pressed
      @return void
      **/
      void actionNewSoundTriggered(void);
      
      /**
      @brief Qt slot that is called when the NewThermal tool button is pressed
      @return void
      **/
      void actionNewThermalTriggered(void);
      
      /**
      @brief Qt slot that is called when the AddRobot tool button is pressed
      @return void
      **/
      void actionAddRobotTriggered(void);
      
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
      
      /**
      @brief Qt slot that is called when the adjusted map visualization tool button is pressed
      @return void
      **/
      void actionAdjustedTriggered(void); 
      
      /**
      @brief Qt slot that is called when the grid status has changed
      @return void
      **/
      void actionGridTriggered(void);
    
    //------------------------------------------------------------------------//
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
      
      /**
      @brief Qt signal that is emmited when the Adjust map button is pressed
      @param state [bool] Toggle flag
      @return void
      **/
      void setAdjustedCursor(bool state);
      
      /**
      @brief Signal emmited on exit pressed
      @return void
      **/
      void guiExitEvent(void);
      
      /**
      @brief Signal emmited on load RFID tag pressed
      @return void
      **/
      void loadRfidPressed(void);
      
      /**
      @brief Signal emmited on load thermal source pressed
      @return void
      **/
      void loadThermalPressed(void);
      
      /**
      @brief Signal emmited on load CO2 source pressed
      @return void
      **/
      void loadCo2Pressed(void);
      
      /**
      @brief Signal emmited on load sound source pressed
      @return void
      **/
      void loadSoundPressed(void);
      
      /**
      @brief Signal emmited on load robot from file pressed
      @param msg [stdr_msgs::RobotMsg] The robot to be placed in the map
      @return void
      **/
      void robotFromFile(stdr_msgs::RobotMsg msg);
  };
}

#endif
