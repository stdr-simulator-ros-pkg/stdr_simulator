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

namespace stdr_gui{
	CGuiLoader::CGuiLoader(int argc,char **argv):
		argc_(argc),
		argv_(argv)
	{
		setupUi(this);
		
		addToolbarIcons();
	}
	
	void CGuiLoader::addToolbarIcons(void)
	{
		
		actionLoadMap = new QAction(this);
        actionLoadMap->setObjectName(QString::fromUtf8("actionLoadMap"));
        actionLoadMap->setCheckable(false);
        actionLoadMap->setIconText(QString("Load map"));
        QIcon iconLoadMap;
        
        
        iconLoadMap.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/load_map.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionLoadMap->setIcon(iconLoadMap);
        toolBar->addAction(actionLoadMap);
		
		toolBar->addSeparator();
        
        actionGrid = new QAction(this);
        actionGrid->setObjectName(QString::fromUtf8("actionGrid"));
        actionGrid->setCheckable(true);
        actionGrid->setChecked(true);
        actionGrid->setIconText(QString("Enable grid"));
        QIcon iconGrid;
        iconGrid.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/grid.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionGrid->setIcon(iconGrid);
        toolBar->addAction(actionGrid);
		
		actionAddRobot = new QAction(this);
        actionAddRobot->setObjectName(QString::fromUtf8("actionNewRobot"));
        actionAddRobot->setCheckable(false);
        actionAddRobot->setIconText(QString("Add robot"));
        QIcon iconAddRobot;
        iconAddRobot.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/add_robot.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionAddRobot->setIcon(iconAddRobot);
        toolBar->addAction(actionAddRobot);
        
        actionNewRobot = new QAction(this);
        actionNewRobot->setObjectName(QString::fromUtf8("actionNewRobot"));
        actionNewRobot->setCheckable(false);
        actionNewRobot->setIconText(QString("Add robot"));
        QIcon iconNewRobot;
        iconNewRobot.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/new_robot.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionNewRobot->setIcon(iconNewRobot);
        toolBar->addAction(actionNewRobot);
        
        actionNewRfid = new QAction(this);
        actionNewRfid->setObjectName(QString::fromUtf8("actionNewRfid"));
        actionNewRfid->setCheckable(false);
        actionNewRfid->setIconText(QString("Add RFID tag"));
        QIcon iconNewRfid;
        iconNewRfid.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/rfid.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionNewRfid->setIcon(iconNewRfid);
        toolBar->addAction(actionNewRfid);
        
        toolBar->addSeparator();
        
        actionProperties = new QAction(this);
        actionProperties->setObjectName(QString::fromUtf8("actionProperties"));
        actionProperties->setCheckable(false);
        actionProperties->setIconText(QString("Properties"));
        QIcon iconProperties;
        iconProperties.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/properties.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionProperties->setIcon(iconProperties);
        toolBar->addAction(actionProperties);
        
        toolBar->addSeparator();
        
        actionZoomIn = new QAction(this);
        actionZoomIn->setObjectName(QString::fromUtf8("actionZoomIn"));
        actionZoomIn->setCheckable(true);
        actionZoomIn->setIconText(QString("Zoom in"));
        QIcon iconZoomIn;
        iconZoomIn.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
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
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/zoom_out_b.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionZoomOut->setIcon(iconZoomOut);
        toolBar->addAction(actionZoomOut);
        
        actionAdjusted = new QAction(this);
        actionAdjusted->setObjectName(QString::fromUtf8("actionAdjusted"));
        actionAdjusted->setCheckable(true);
        actionAdjusted->setChecked(true);
        actionAdjusted->setIconText(QString("Auto size"));
        QIcon iconAdjust;
        iconAdjust.addFile(QString::fromUtf8((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
				std::string("/resources/images/adjusted.png")).c_str()), 
			QSize(), 
			QIcon::Normal, 
			QIcon::Off);
        actionAdjusted->setIcon(iconAdjust);
        toolBar->addAction(actionAdjusted);
        
        toolBar->setIconSize(QSize(30,30));
	}
	
	void CGuiLoader::closeEvent(QCloseEvent *event)
	{
		ros::shutdown();
	}
}
