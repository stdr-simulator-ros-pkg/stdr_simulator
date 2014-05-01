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

#include "stdr_gui/stdr_gui_connector.h"

namespace stdr_gui
{
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CGuiConnector::CGuiConnector(int argc, char **argv):
    QObject(),
    loader_(argc,argv),
    robotCreatorConn(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    
    bool map_initialized_ = false;
    
    QObject::connect(
      loader_.actionProperties,SIGNAL(triggered(bool)),
      this,SLOT(actionPropertiesTriggered()));
    
    QObject::connect(
      loader_.actionAbout_STDR_Simulator,SIGNAL(triggered(bool)),
      this,SLOT(actionAboutTriggered()));
    
    QObject::connect(
      loader_.actionExit,SIGNAL(triggered(bool)),
      this,SLOT(actionExitTriggered()));
      
    QObject::connect(
      loader_.actionLoadMap,SIGNAL(triggered(bool)),
      this,SLOT(actionLoadMapTriggered()));
    
    QObject::connect(
      loader_.actionNewRobot,SIGNAL(triggered(bool)),
      this,SLOT(actionNewRobotTriggered()));
      
    QObject::connect(
      loader_.actionAddRobot,SIGNAL(triggered(bool)),
      this,SLOT(actionAddRobotTriggered()));
      
    QObject::connect(
      loader_.actionZoomIn,SIGNAL(triggered(bool)),
      this,SLOT(actionZoomInTriggered()));
    
    QObject::connect(
      loader_.actionZoomOut,SIGNAL(triggered(bool)),
      this,SLOT(actionZoomOutTriggered()));
    
    QObject::connect( 
      loader_.actionAdjusted,SIGNAL(triggered(bool)),
      this,SLOT(actionAdjustedTriggered()));
    
    QObject::connect(
      loader_.actionGrid,SIGNAL(triggered(bool)),
      this,SLOT(actionGridTriggered()));
      
    QObject::connect(
      loader_.actionNewRfid,SIGNAL(triggered(bool)),
      this,SLOT(actionNewRfidTriggered()));
      
    QObject::connect(
      loader_.actionNewThermal,SIGNAL(triggered(bool)),
      this,SLOT(actionNewThermalTriggered()));
      
    QObject::connect(
      loader_.actionNewCo2,SIGNAL(triggered(bool)),
      this,SLOT(actionNewCo2Triggered()));
      
    QObject::connect(
      loader_.actionNewSound,SIGNAL(triggered(bool)),
      this,SLOT(actionNewSoundTriggered()));
    
    grid_enabled_ = false;
  }
  
  /**
  @brief Qt slot that is called when the Exit action is triggered
  @return void
  **/
  void CGuiConnector::actionExitTriggered(void)
  {
    ROS_INFO("Exiting GUI...");
    exit(0);
  }

  /**
  @brief Qt slot that is called when the Properties tool button is pressed
  @return void
  **/
  void CGuiConnector::actionPropertiesTriggered(void)
  {
    QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
    msg.setWindowTitle(QString("Not finished yet :/"));
    msg.exec();
  }
  
  /**
  @brief Qt slot that is called when the LoadMap tool button is pressed
  @return void
  **/
  void CGuiConnector::actionLoadMapTriggered(void)
  {
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load map"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources") + 
          std::string("/maps")), 
        tr("Yaml map files (*.yaml)"));
    if(file_name.isEmpty() || file_name.isNull())
    {
      return;
    }
    
    ros::NodeHandle nh;
    
    nav_msgs::OccupancyGrid map;
    
    map = stdr_server::map_loader::loadMap(file_name.toStdString().c_str());
    
    ros::ServiceClient client;
    
    while (!ros::service::waitForService
      ("/stdr_server/load_static_map_external", ros::Duration(.1)) && 
        ros::ok()) 
    {
      ROS_WARN
        ("Trying to register to /stdr_server/load_static_map_external...");
    }
    
    client = nh.serviceClient<stdr_msgs::LoadExternalMap>
      ("/stdr_server/load_static_map_external", true);
    
    stdr_msgs::LoadExternalMap srv;
    
    srv.request.map = map;
    
    if (client.call(srv)) {
      ROS_INFO("Map successfully loaded");
    }
    else {
      ROS_ERROR("Could not load map, maybe already loaded...");
    }
  }
  
  /**
  @brief Qt slot that is called when the About tool button is pressed
  @return void
  **/
  void CGuiConnector::actionAboutTriggered(void)
  {
    QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
    msg.setWindowTitle(QString("STDR Simulator - About"));
    msg.setText(QString("Simple Two Dimentional Robot Simulator\
 (STDR Simulator) is a multi-robot simulator created in QT4. Its goals\
 are : \n\n1) to simulate easily a single robot or a swarm in a 2D\
 environment, \n2) to be totally parameterizable \n3) to be ROS\
 compliant.\n\nDevelopers:\nManos Tsardoulias, etsardou@gmail.com\
 \nChris Zalidis,zalidis@gmail.com\nAris Thallas, aris.thallas@gmail.com"));
    msg.exec();
  }
  
  /**
  @brief Qt slot that is called when the NewRobot tool button is pressed
  @return void
  **/
  void CGuiConnector::actionNewRobotTriggered(void)
  {
    robotCreatorConn.initialise();
  }
  
  /**
  @brief Qt slot that is called when the AddRobot tool button is pressed
  @return void
  **/
  void CGuiConnector::actionAddRobotTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load robot"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Robot Files (*.yaml *xml)"));
    
    if (file_name.isEmpty()) { //!< Not a valid filename
      return;
    }
    
    try {
      stdr_msgs::RobotMsg new_robot_msg = 
        stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>
          (file_name.toStdString());
          
      Q_EMIT robotFromFile(new_robot_msg);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
  }
  
  /**
  @brief Qt slot that is called when the NewRfid tool button is pressed
  @return void
  **/
  void CGuiConnector::actionNewRfidTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT loadRfidPressed();
  }
  
  /**
  @brief Qt slot that is called when the NewThermal tool button is pressed
  @return void
  **/
  void CGuiConnector::actionNewThermalTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT loadThermalPressed();
  }
  
  /**
  @brief Qt slot that is called when the NewCO2 tool button is pressed
  @return void
  **/
  void CGuiConnector::actionNewCo2Triggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT loadCo2Pressed();
  }
  
  /**
  @brief Qt slot that is called when the NewSound tool button is pressed
  @return void
  **/
  void CGuiConnector::actionNewSoundTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT loadSoundPressed();
  }
  
  /**
  @brief Qt slot that is called when the zoom in tool button is pressed
  @return void
  **/
  void CGuiConnector::actionZoomInTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT setZoomInCursor(loader_.actionZoomIn->isChecked());
    loader_.actionZoomOut->setChecked(false);
    loader_.actionAdjusted->setChecked(false);
  }
  
  /**
  @brief Qt slot that is called when the zoom out tool button is pressed
  @return void
  **/
  void CGuiConnector::actionZoomOutTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT setZoomOutCursor(loader_.actionZoomOut->isChecked());
    loader_.actionZoomIn->setChecked(false);
    loader_.actionAdjusted->setChecked(false);
  }
  
  /**
  @brief Qt slot that is called when the adjusted map visualization tool button is pressed
  @return void
  **/
  void CGuiConnector::actionAdjustedTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT setAdjustedCursor(loader_.actionAdjusted->isChecked());
    loader_.actionZoomIn->setChecked(false);
    loader_.actionZoomOut->setChecked(false);
  }
  
  /**
  @brief Qt slot that is called when the grid status has changed
  @return void
  **/
  void CGuiConnector::actionGridTriggered(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    grid_enabled_ =! grid_enabled_;
  }
  
  /**
  @brief Returns the grid enabled state
  @return bool : True if grid is enabled
  **/
  bool CGuiConnector::isGridEnabled(void)
  {
    return grid_enabled_;
  }
  
  /**
  @brief Adds a widget to the main window Qt grid
  @param w [QWidget*] The widget to be placed
  @param row [int] The row of the grid
  @param column [int] The column of the grid
  @return void
  **/
  void CGuiConnector::addToGrid(QWidget *w,int row,int column)
  {
    loader_.gridLayout->addWidget(w,row,column,0);  
  }
  
  /**
  @brief Wraps the Qt gridColumnStretch function
  @param cell [int] The specific column
  @param stretch [int] The relative stretch coefficient
  @return void
  **/
  void CGuiConnector::setGridColumnStretch(int cell,int stretch)
  {
    loader_.gridLayout->setColumnStretch(cell,stretch);
  }
  
  /**
  @brief Shows the main window
  @return void
  **/
  void CGuiConnector::show(void)
  {
    loader_.show();
    loader_.showMaximized();
  }
  
  /**
  @brief Displays a message in the QMainWindow status bar
  @param s [QString] The message
  @return void
  **/
  void CGuiConnector::setStatusBarMessage(QString s)
  {
    loader_.statusbar->showMessage(s,0);
  }
  
  /**
  @brief Returns the exit event captured
  @return QEvent* The captured event
  **/
  QEvent* CGuiConnector::getCloseEvent(void)
  {
    return loader_.getCloseEvent();
  }
    
  /**
  @brief Returns the exit triggered status
  @return bool True if exit has been triggered
  **/
  bool CGuiConnector::closeTriggered(void)
  {
    return loader_.closeTriggered();
  }
  
  /**
  @brief Shuts down the main window
  @return void
  **/
  void CGuiConnector::shutdown(void)
  {
    loader_.shutdown();
  }
  
  /**
  @brief Sets the map_initialized_ private variable
  @param mi [bool] The new value
  @return void
  **/
  void CGuiConnector::setMapInitialized(bool mi)
  {
    map_initialized_ = mi;
  }
  
  /**
  @brief Unchecks the zoom in & out buttons when right click in map is pushed
  @return void
  **/
  void CGuiConnector::uncheckZoomButtons(void)
  {
    loader_.actionZoomIn->setChecked(false);
    loader_.actionZoomOut->setChecked(false);
  }
  
  /**
  @brief Raises a message box with a specific message
  @param title [QString] The message box title
  @param s [QString] The message
  @return void
  **/
  void CGuiConnector::raiseMessage(QString title, QString s)
  {
    QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
    msg.setWindowTitle(title);
    msg.setText(s);
    msg.exec();
  }
}
