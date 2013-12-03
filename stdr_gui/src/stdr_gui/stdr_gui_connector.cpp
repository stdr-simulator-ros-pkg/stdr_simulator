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
	CGuiConnector::CGuiConnector(int argc, char **argv):
		QObject(),
		loader_(argc,argv),
		robotCreatorConn(argc,argv),
		argc_(argc),
		argv_(argv)
	{
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
		
		grid_enabled_=true;
	}
	
	void CGuiConnector::actionExitTriggered(void)
	{
		ROS_INFO("Exiting GUI...");
		exit(0);
	}

	void CGuiConnector::actionPropertiesTriggered(void)
	{
		QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
		msg.setWindowTitle(QString("Not finished yet :/"));
		msg.exec();
	}
	
	void CGuiConnector::actionLoadMapTriggered(void)
	{
		QString fileName = QFileDialog::getOpenFileName(
			&loader_,
			tr("Load map"), 
			QString().fromStdString(getRosPackagePath("stdr_gui")), 
			tr("Yaml Files (*.yaml)"));
	}
	
	void CGuiConnector::actionAboutTriggered(void)
	{
		QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
		msg.setWindowTitle(QString("STDR Simulator - About"));
		msg.setText(QString("Simple Two Dimentional Robot Simulator \
		(STDR Simulator) is a multi-robot simulator created in QT4. Its goals \
		are : \n1) to simulate easily a single robot or a swarm in a 2D \
		environment, \n2) to be totally parameterizable \n3) to be ROS \
		compliant.\n\nDevelopers:\nManos Tsardoulias, etsardou@gmail.com\
		\nAris Thallas, aris.thallas@gmail.com\nChris Zalidis, \
		zalidis@gmail.com"));
		msg.exec();
	}
	
	void CGuiConnector::actionNewRobotTriggered(void)
	{
		robotCreatorConn.initialise();
	}
	
	void CGuiConnector::setMapLoaded(bool mapLoaded)
	{
		map_loaded_=mapLoaded;
	}
	
	void CGuiConnector::actionZoomInTriggered(void)
	{
		//~ if(!_mapLoaded) return;
		Q_EMIT setZoomInCursor(loader_.actionZoomIn->isChecked());
		loader_.actionZoomOut->setChecked(false);
		loader_.actionAdjusted->setChecked(false);
	}
	void CGuiConnector::actionZoomOutTriggered(void)
	{
		//~ if(!_mapLoaded) return;
		Q_EMIT setZoomOutCursor(loader_.actionZoomOut->isChecked());
		loader_.actionZoomIn->setChecked(false);
		loader_.actionAdjusted->setChecked(false);
	}
	
	void CGuiConnector::actionAdjustedTriggered(void)
	{
		//~ if(!_mapLoaded) return;
		Q_EMIT setAdjustedCursor(loader_.actionAdjusted->isChecked());
		loader_.actionZoomIn->setChecked(false);
		loader_.actionZoomOut->setChecked(false);
	}
	
	void CGuiConnector::actionGridTriggered(void)
	{
		grid_enabled_=!grid_enabled_;
	}
	
	bool CGuiConnector::isGridEnabled(void)
	{
		return grid_enabled_;
	}
	
	void CGuiConnector::addToGrid(QWidget *w,int row,int column)
	{
		loader_.gridLayout->addWidget(w,row,column,0);	
	}
	
	void CGuiConnector::setGridColumnStretch(int cell,int stretch)
	{
		loader_.gridLayout->setColumnStretch(cell,stretch);
	}
	
	void CGuiConnector::show(void)
	{
		loader_.show();
	}
	
	void CGuiConnector::setStatusBarMessage(QString s)
	{
		loader_.statusbar->showMessage(s,0);
	}
}
