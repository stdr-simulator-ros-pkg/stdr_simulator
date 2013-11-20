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

#include "stdr_gui/stdr_gui_controller.h"


namespace stdr{
	GuiController::GuiController(int argc,char **argv):
			guiConnector(argc,argv),
			infoConnector(argc,argv),
			mapConnector(argc,argv)
	{
		this->argc=argc;
		this->argv=argv;
		
		setupWidgets();
	}
	
	void GuiController::setupWidgets(void){

		{
			QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
			sizePolicy.setHorizontalStretch(0);
			sizePolicy.setVerticalStretch(0);
			infoConnector.loader.setSizePolicy(sizePolicy);
			infoConnector.loader.setMaximumSize(QSize(300, 8000));
			guiConnector.loader.gridLayout->addWidget(static_cast<QWidget *>(&infoConnector.loader),0,0,0);	
		}
		
		{
			QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
			sizePolicy.setHorizontalStretch(0);
			sizePolicy.setVerticalStretch(0);
			mapConnector.loader.setSizePolicy(sizePolicy);
			QImage logo((getRosPackagePath("stdr_gui")+std::string("/resources/images/logo.png")).c_str());
			QPainter painter(&logo);
			mapConnector.loader.map->setScaledContents(true);
			mapConnector.loader.map->setPixmap(QPixmap().fromImage(logo));
			guiConnector.loader.gridLayout->addWidget(static_cast<QWidget *>(&mapConnector.loader),0,1,0);	
		}
		
	}
	
	bool GuiController::init(void){
		ros::init(argc,argv,"stdr_gui_node");
		
		if ( ! ros::master::check() ) {
			return false;
		}
		ros::start();

		guiConnector.loader.show();
		start();
		
		initializeCommunications();
		return true;
	}
	
	void GuiController::initializeCommunications(void){
		ros::NodeHandle n;
		mapSubscriber=n.subscribe("map", 1, &GuiController::receiveMap,this);
	}
	
	void GuiController::receiveMap(const nav_msgs::OccupancyGrid& msg){
		mapMsg=msg;
		initialMap=QImage(msg.info.width,msg.info.height,QImage::Format_RGB32);
		QPainter painter(&initialMap);
		int d=0;
		QColor c;
		for(unsigned int i=0;i<msg.info.width;i++){
			for(unsigned int j=0;j<msg.info.height;j++){
				if(msg.data[j*msg.info.width+i]==-1)
					c=QColor(127,127,127);
				else{
					d=(100.0-msg.data[j*msg.info.width+i])/100.0*255.0;
					c=QColor(d,d,d);
				}
				painter.setPen(c);
				painter.drawPoint(i,j);
			}
		}	
	}
}

