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

namespace stdr_gui
{
  class CRobotCreatorConnector:
    public QObject
  {
    Q_OBJECT
    
    private:
      int   argc_; 
      char**  argv_;
      float climax_;
      
      CRobotCreatorLoader loader_;
      
      stdr_msgs::RobotMsg new_robot_msg_;
      
      QTreeWidgetItem*   current_laser_;
      QTreeWidgetItem*  current_sonar_;
      QTreeWidgetItem*  current_rfid_;
      
    public:
      static unsigned int laser_number;
      static unsigned int sonar_number;
      static unsigned int rfid_number;
    
      CRobotCreatorConnector(int argc, char **argv);
      ~CRobotCreatorConnector(void);
      
      void initialise(void);
      void deleteTreeNode(QTreeWidgetItem *item);
      
      void editRobot(void);
      
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
      
      void setInitialPose(float x, float y);
      void fixRobotMsgAngles(void);
      
      stdr_msgs::RobotMsg getNewRobot(void);
      
    public Q_SLOTS:
      void treeItemClicked ( QTreeWidgetItem * item, int column ); 
      void updateLaser(void);
      void updateSonar(void);
      void updateRfid(void);
      void updateRobot(void);
      void saveRobot(void);
      void closeRobotCreator(void);
      void loadRobot(void);
      
    Q_SIGNALS:
      void loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
      void saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
  };
}

#endif
