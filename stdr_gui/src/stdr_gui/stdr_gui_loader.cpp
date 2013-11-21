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

namespace stdr{
	GuiLoader::GuiLoader(int argc,char **argv){
		this->argc=argc;
		this->argv=argv;
		
		setupUi(this);
		
		addToolbarIcons();
	}
	
	void GuiLoader::addToolbarIcons(void){
		
		actionLoadMap = new QAction(this);
        actionLoadMap->setObjectName(QString::fromUtf8("actionLoadMap"));
        actionLoadMap->setCheckable(false);
        actionLoadMap->setIconText(QString("Load map"));
        QIcon iconLoadMap;
        
        
        iconLoadMap.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/orange-moonlight-icons/png/32x32/image.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionLoadMap->setIcon(iconLoadMap);
        toolBar->addAction(actionLoadMap);
		
		toolBar->addSeparator();
        
        actionGrid = new QAction(this);
        actionGrid->setObjectName(QString::fromUtf8("actionGrid"));
        actionGrid->setCheckable(false);
        actionGrid->setIconText(QString("Enable grid"));
        QIcon iconGrid;
        iconGrid.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/orange-moonlight-icons/png/32x32/grid.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionGrid->setIcon(iconGrid);
        toolBar->addAction(actionGrid);
		
		actionNewRobot = new QAction(this);
        actionNewRobot->setObjectName(QString::fromUtf8("actionNewRobot"));
        actionNewRobot->setCheckable(false);
        actionNewRobot->setIconText(QString("Add robot"));
        QIcon iconNewRobot;
        iconNewRobot.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/orange-moonlight-icons/png/32x32/blank_page.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionNewRobot->setIcon(iconNewRobot);
        toolBar->addAction(actionNewRobot);
        
        actionNewRfid = new QAction(this);
        actionNewRfid->setObjectName(QString::fromUtf8("actionNewRfid"));
        actionNewRfid->setCheckable(false);
        actionNewRfid->setIconText(QString("Add RFID tag"));
        QIcon iconNewRfid;
        iconNewRfid.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/orange-moonlight-icons/png/32x32/target.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionNewRfid->setIcon(iconNewRfid);
        toolBar->addAction(actionNewRfid);
        
        toolBar->addSeparator();
        
        actionProperties = new QAction(this);
        actionProperties->setObjectName(QString::fromUtf8("actionProperties"));
        actionProperties->setCheckable(false);
        actionProperties->setIconText(QString("Properties"));
        QIcon iconProperties;
        iconProperties.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/orange-moonlight-icons/png/32x32/process.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionProperties->setIcon(iconProperties);
        toolBar->addAction(actionProperties);
        
        toolBar->addSeparator();
        
        actionZoomIn = new QAction(this);
        actionZoomIn->setObjectName(QString::fromUtf8("actionZoomIn"));
        actionZoomIn->setCheckable(true);
        actionZoomIn->setIconText(QString("Zoom in"));
        QIcon iconZoomIn;
        iconZoomIn.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/zoom_in.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionZoomIn->setIcon(iconZoomIn);
        toolBar->addAction(actionZoomIn);
        
        actionZoomOut = new QAction(this);
        actionZoomOut->setObjectName(QString::fromUtf8("actionZoomIn"));
        actionZoomOut->setCheckable(true);
        actionZoomOut->setIconText(QString("Zoom in"));
        QIcon iconZoomOut;
        iconZoomOut.addFile(QString::fromUtf8((getRosPackagePath("stdr_gui")+std::string("/resources/images/zoom_out.png")).c_str()), QSize(), QIcon::Normal, QIcon::Off);
        actionZoomOut->setIcon(iconZoomOut);
        toolBar->addAction(actionZoomOut);
	}
}
