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

#ifndef STDR_ROBOT_CREATOR_CONNECTOR
#define STDR_ROBOT_CREATOR_CONNECTOR

#include "stdr_gui/stdr_robot_creator/stdr_robot_creator_loader.h"
#include "stdr_msgs/RobotMsg.h"

namespace stdr{
	class RobotCreatorConnector:public QObject{
		Q_OBJECT
		
			int argc; 
			char **argv;
			
		public:
			RobotCreatorConnector(int argc, char **argv);
			RobotCreatorLoader loader;

			stdr_msgs::RobotMsg newRobotMsg;
			
			QTreeWidgetItem *currentLaser;
			QTreeWidgetItem *currentSonar;
			QTreeWidgetItem *currentRfid;
			float climax;
			
			void initialise(void);
			void deleteTreeNode(QTreeWidgetItem *item);
			
			void addLaser(void);
			void eraseLaser(QTreeWidgetItem *item);
			void editLaser(QTreeWidgetItem *item);
			int searchLaser(QString frameId);
			
			void addSonar(void);
			void eraseSonar(QTreeWidgetItem *item);
			void editSonar(QTreeWidgetItem *item);
			int searchSonar(QString frameId);
			
			void addRfidAntenna(void);
			void eraseRfid(QTreeWidgetItem *item);
			void editRfid(QTreeWidgetItem *item);
			int searchRfid(QString frameId);
			
			void drawRobot(float radius);
			void drawRobot(float length,float width);
			void drawRobot(std::vector<std::pair<float,float> > geometry);
			void drawLasers(void);
			void drawSonars(void);
			void drawRfidAntennas(void);

			void updateRobotPreview(void);
			
			void printLaserMsg(stdr_msgs::LaserSensorMsg msg);
			
			static unsigned int laserNumber;
			static unsigned int sonarNumber;
			static unsigned int rfidNumber;

		public Q_SLOTS:
			void treeItemClicked ( QTreeWidgetItem * item, int column ); 
			void updateLaser(void);
			void updateSonar(void);
			void updateRfid(void);
			void saveRobot(void);
			void closeRobotCreator(void);
	};
}

#endif
