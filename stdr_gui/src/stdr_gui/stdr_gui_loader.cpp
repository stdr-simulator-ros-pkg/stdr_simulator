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

#include "stdr_gui/stdr_gui_loader.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CGuiLoader::CGuiLoader(int argc,char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    
    addToolbarIcons();
    close_signal_ = false;
  }
  
  /**
  @brief Adds the tool buttons in the main window toolbar
  @return void
  **/
  void CGuiLoader::addToolbarIcons(void)
  {
    
    QIcon applicationIcon;
    applicationIcon.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/favicon.ico")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    setWindowIcon (applicationIcon);
    
    actionLoadMap = new QAction(this);
    actionLoadMap->setObjectName(QString::fromUtf8("actionLoadMap"));
    actionLoadMap->setCheckable(false);
    actionLoadMap->setIconText(QString("Load map"));
    QIcon iconLoadMap;
        
        
    iconLoadMap.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/load_map.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionLoadMap->setIcon(iconLoadMap);
    toolBar->addAction(actionLoadMap);
    
    toolBar->addSeparator();
    
    actionAddRobot = new QAction(this);
    actionAddRobot->setObjectName(QString::fromUtf8("actionNewRobot"));
    actionAddRobot->setCheckable(false);
    actionAddRobot->setIconText(QString("Load robot"));
    QIcon iconAddRobot;
    iconAddRobot.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/add_robot.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionAddRobot->setIcon(iconAddRobot);
    toolBar->addAction(actionAddRobot);
        
    actionNewRobot = new QAction(this);
    actionNewRobot->setObjectName(QString::fromUtf8("actionNewRobot"));
    actionNewRobot->setCheckable(false);
    actionNewRobot->setIconText(QString("Create robot"));
    QIcon iconNewRobot;
    iconNewRobot.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/new_robot.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionNewRobot->setIcon(iconNewRobot);
    toolBar->addAction(actionNewRobot);
        
    toolBar->addSeparator();
        
    actionNewRfid = new QAction(this);
    actionNewRfid->setObjectName(QString::fromUtf8("actionNewRfid"));
    actionNewRfid->setCheckable(false);
    actionNewRfid->setIconText(QString("Add RFID tag"));
    QIcon iconNewRfid;
    iconNewRfid.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/rfid.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionNewRfid->setIcon(iconNewRfid);
    toolBar->addAction(actionNewRfid);
        
    actionNewThermal = new QAction(this);
    actionNewThermal->setObjectName(QString::fromUtf8("actionNewThermal"));
    actionNewThermal->setCheckable(false);
    actionNewThermal->setIconText(QString("Add heat source"));
    QIcon iconNewThermal;
    iconNewThermal.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/thermal.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    
    actionNewThermal->setIcon(iconNewThermal);
    toolBar->addAction(actionNewThermal);
        
    actionNewCo2 = new QAction(this);
    actionNewCo2->setObjectName(QString::fromUtf8("actionNewCo2"));
    actionNewCo2->setCheckable(false);
    actionNewCo2->setIconText(QString("Add CO2 source"));
    QIcon iconNewCo2;
    iconNewCo2.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/co2.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionNewCo2->setIcon(iconNewCo2);
    toolBar->addAction(actionNewCo2);
    
    actionNewSound = new QAction(this);
    actionNewSound->setObjectName(QString::fromUtf8("actionNewSound"));
    actionNewSound->setCheckable(false);
    actionNewSound->setIconText(QString("Add sound source"));
    QIcon iconNewSound;
    iconNewSound.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/sound.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionNewSound->setIcon(iconNewSound);
    toolBar->addAction(actionNewSound);
        
    toolBar->addSeparator();
        
    actionProperties = new QAction(this);
    actionProperties->setObjectName(QString::fromUtf8("actionProperties"));
    actionProperties->setCheckable(false);
    actionProperties->setIconText(QString("Properties"));
    QIcon iconProperties;
    iconProperties.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/properties.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionProperties->setIcon(iconProperties);
    toolBar->addAction(actionProperties);
        
    toolBar->addSeparator();
        
    actionGrid = new QAction(this);
    actionGrid->setObjectName(QString::fromUtf8("actionGrid"));
    actionGrid->setCheckable(true);
    actionGrid->setChecked(false);
    actionGrid->setIconText(QString("Enable grid"));
    QIcon iconGrid;
    iconGrid.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/grid.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionGrid->setIcon(iconGrid);
    toolBar->addAction(actionGrid);
        
    actionZoomIn = new QAction(this);
    actionZoomIn->setObjectName(QString::fromUtf8("actionZoomIn"));
    actionZoomIn->setCheckable(true);
    actionZoomIn->setIconText(QString("Zoom in"));
    QIcon iconZoomIn;
    iconZoomIn.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/zoom_in_b.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionZoomIn->setIcon(iconZoomIn);
    toolBar->addAction(actionZoomIn);
        
    actionZoomOut = new QAction(this);
    actionZoomOut->setObjectName(QString::fromUtf8("actionZoomOut"));
    actionZoomOut->setCheckable(true);
    actionZoomOut->setIconText(QString("Zoom out"));
    QIcon iconZoomOut;
    iconZoomOut.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/zoom_out_b.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionZoomOut->setIcon(iconZoomOut);
    toolBar->addAction(actionZoomOut);
        
    actionAdjusted = new QAction(this);
    actionAdjusted->setObjectName(QString::fromUtf8("actionAdjusted"));
    actionAdjusted->setCheckable(false);
    actionAdjusted->setChecked(true);
    actionAdjusted->setIconText(QString("Adjust size"));
    QIcon iconAdjust;
    iconAdjust.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/adjusted.png")).c_str()), 
      QSize(), 
      QIcon::Normal, 
      QIcon::Off);
    actionAdjusted->setIcon(iconAdjust);
    toolBar->addAction(actionAdjusted);
        
    toolBar->setIconSize(QSize(30,30));
  }
  
  /**
  @brief Overloading of closeEvent function from QMainWindow
  @param event [QCloseEvent*] The exit event
  @return void
  **/
  void CGuiLoader::closeEvent(QCloseEvent *event)
  {
    //~ ROS_ERROR("Shutdown signal!");
    if(close_signal_)
    {
      event->accept();
      //~ ROS_ERROR("Shutting down ros...");
      ros::shutdown();
      exit(0);
      return;
    }
    close_signal_ = true;
    event->ignore();
    event_ = event;
  }
  
  /**
  @brief Returns the exit event
  @return QEvent* 
  **/
  QEvent* CGuiLoader::getCloseEvent(void)
  {
    return event_;
  }
  
  /**
  @brief Returns true if a close event was triggered
  @return bool
  **/
  bool CGuiLoader::closeTriggered(void)
  {
    return close_signal_;
  }
  
  /**
  @brief Shuts down the main window
  @return void
  **/
  void CGuiLoader::shutdown(void)
  {
    this->close();
  }
}
