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

namespace stdr_gui{
	GuiConnector::GuiConnector(int argc, char **argv):
		QObject(),
		loader(argc,argv),
		robotCreatorConn(argc,argv)
	{
		this->argc=argc;
		this->argv=argv;
		
		QObject::connect(loader.actionProperties,SIGNAL(triggered(bool)),this,SLOT(actionPropertiesTriggered()));
		QObject::connect(loader.actionAbout_STDR_Simulator,SIGNAL(triggered(bool)),this,SLOT(actionAboutTriggered()));
		QObject::connect(loader.actionExit,SIGNAL(triggered(bool)),this,SLOT(actionExitTriggered()));
		QObject::connect(loader.actionLoadMap,SIGNAL(triggered(bool)),this,SLOT(actionLoadMapTriggered()));
		QObject::connect(loader.actionNewRobot,SIGNAL(triggered(bool)),this,SLOT(actionNewRobotTriggered()));
		QObject::connect(loader.actionZoomIn,SIGNAL(triggered(bool)),this,SLOT(actionZoomInTriggered()));
		QObject::connect(loader.actionZoomOut,SIGNAL(triggered(bool)),this,SLOT(actionZoomOutTriggered()));
		QObject::connect(loader.actionRealSize,SIGNAL(triggered(bool)),this,SLOT(actionRealSizeTriggered()));
		QObject::connect(loader.actionAdjusted,SIGNAL(triggered(bool)),this,SLOT(actionAdjustedTriggered()));
	}
	
	void GuiConnector::actionExitTriggered(void){
		ROS_INFO("Exiting GUI...");
		exit(0);
	}

	void GuiConnector::actionPropertiesTriggered(void){
		QMessageBox msg(static_cast<QMainWindow *>(&this->loader));
		msg.setWindowTitle(QString("Not finished yet :/"));
		msg.exec();
	}
	
	void GuiConnector::actionLoadMapTriggered(void){
		QString fileName = QFileDialog::getOpenFileName(&loader,tr("Load map"), QString().fromStdString(getRosPackagePath("stdr_gui")), tr("Yaml Files (*.yaml)"));
	}
	
	void GuiConnector::actionAboutTriggered(void){
		QMessageBox msg(static_cast<QMainWindow *>(&this->loader));
		msg.setWindowTitle(QString("STDR Simulator - About"));
		msg.setText(QString("Simple Two Dimentional Robot Simulator (STDR Simulator) is a multi-robot simulator created in QT4. Its goals are : \n1) to simulate easily a single robot or a swarm in a 2D environment, \n2) to be totally parameterizable \n3) to be ROS compliant.\n\nDevelopers:\nManos Tsardoulias, etsardou@gmail.com\nAris Thallas, aris.thallas@gmail.com\nChris Zalidis, zalidis@gmail.com"));
		msg.exec();
	}
	
	void GuiConnector::actionNewRobotTriggered(void){
		robotCreatorConn.initialise();
	}
	
	void GuiConnector::setMapLoaded(bool mapLoaded){
		_mapLoaded=mapLoaded;
	}
	
	void GuiConnector::actionZoomInTriggered(void){
		//~ if(!_mapLoaded) return;
		Q_EMIT setZoomInCursor(loader.actionZoomIn->isChecked());
		loader.actionZoomOut->setChecked(false);
		loader.actionRealSize->setChecked(false);
		loader.actionAdjusted->setChecked(false);
	}
	void GuiConnector::actionZoomOutTriggered(void){
		//~ if(!_mapLoaded) return;
		Q_EMIT setZoomOutCursor(loader.actionZoomOut->isChecked());
		loader.actionZoomIn->setChecked(false);
		loader.actionRealSize->setChecked(false);
		loader.actionAdjusted->setChecked(false);
	}
	
	void GuiConnector::actionRealSizeTriggered(void){
		//~ if(!_mapLoaded) return;
		Q_EMIT setRealSizeCursor(loader.actionRealSize->isChecked());
		loader.actionZoomIn->setChecked(false);
		loader.actionZoomOut->setChecked(false);
		loader.actionAdjusted->setChecked(false);
	}
	
	void GuiConnector::actionAdjustedTriggered(void){
		//~ if(!_mapLoaded) return;
		Q_EMIT setRealSizeCursor(loader.actionRealSize->isChecked());
		loader.actionZoomIn->setChecked(false);
		loader.actionZoomOut->setChecked(false);
		loader.actionRealSize->setChecked(false);
	}
}
