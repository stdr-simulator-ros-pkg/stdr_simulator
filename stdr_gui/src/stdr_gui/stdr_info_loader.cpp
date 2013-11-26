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

#include "stdr_gui/stdr_info_loader.h"

namespace stdr_gui{
	InfoLoader::InfoLoader(int argc, char **argv){
		this->argc=argc;
		this->argv=argv;
		
		setupUi(this);
		
		stdrInformationTree->setColumnCount(2);
		stdrInformationTree->setColumnWidth(0,200);
		
		generalInfo.setText(0,"Simulation information");
		robotsInfo.setText(0,"Robots");
		
		mapWidth.setText(0,"Map width");
		mapWidth.setText(1,"-");
		generalInfo.addChild(&mapWidth);
		mapHeight.setText(0,"Map height");
		mapHeight.setText(1,"-");
		generalInfo.addChild(&mapHeight);
		mapOcgd.setText(0,"Resolution");
		mapOcgd.setText(1,"-");
		generalInfo.addChild(&mapOcgd);
		
		stdrInformationTree->addTopLevelItem(&generalInfo);
		stdrInformationTree->addTopLevelItem(&robotsInfo);
		
		generalInfo.setExpanded(true);
		robotsInfo.setExpanded(true);
	}
	
	void InfoLoader::deleteTreeNode(QTreeWidgetItem *item){
		int count=item->childCount();
		for(int i=count-1;i>=0;i--)
			deleteTreeNode(item->child(i));
		delete item;
	}
	
	void InfoLoader::deleteTree(void){
		for(unsigned int i=0;i<robotsInfo.childCount();i++){
			deleteTreeNode(robotsInfo.child(i));
		}
	}
	
	void InfoLoader::updateMapInfo(float width,float height,float ocgd){
		mapWidth.setText(1,(QString().setNum(width)+QString(" m")));
		mapHeight.setText(1,(QString().setNum(height)+QString(" m")));
		mapOcgd.setText(1,(QString().setNum(ocgd)+QString(" m/px")));
	}
	
	void InfoLoader::updateRobots(const stdr_msgs::RobotIndexedVectorMsg& msg){
		for(unsigned int i=0;i<msg.robots.size();i++){
			QTreeWidgetItem	*rnode;
			rnode=new QTreeWidgetItem();
			rnode->setText(0,QString(msg.robots[i].name.c_str()));
			
			robotsInfo.addChild(rnode);
		}
	}
}
