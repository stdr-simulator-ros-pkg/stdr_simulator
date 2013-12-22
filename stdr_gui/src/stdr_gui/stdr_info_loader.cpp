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
  CInfoLoader::CInfoLoader(int argc, char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    
    stdrInformationTree->header()->setDefaultSectionSize(20);
    stdrInformationTree->header()->setMinimumSectionSize(10);
    
    stdrInformationTree->setColumnCount(4);
    stdrInformationTree->setColumnWidth(0,200);
    stdrInformationTree->setColumnWidth(1,100);
    stdrInformationTree->setColumnWidth(2,20);
    stdrInformationTree->setColumnWidth(3,20);
    
    QStringList ColumnNames;
    ColumnNames << "" << "" << "" << "" << "";
 
    stdrInformationTree->setHeaderLabels(ColumnNames);
    
    generalInfo.setText(0,"Information");
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

    visible_icon_.addFile(QString((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/visible.png")).c_str()));
  }
  
  CInfoLoader::~CInfoLoader(void)
  {
    
  }
  
  void CInfoLoader::deleteTreeNode(QTreeWidgetItem *item)
  {
    int count = item->childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      deleteTreeNode(item->child(i));
    }
    stdrInformationTree->removeItemWidget(item,0);
    delete item;
  }
  
  void CInfoLoader::deleteTree(void)
  {
    int count = robotsInfo.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      deleteTreeNode(robotsInfo.child(i));
    }
  }
  
  void CInfoLoader::updateMapInfo(float width,float height,float ocgd)
  {
    mapWidth.setText(1,(QString().setNum(width) + QString(" m")));
    mapHeight.setText(1,(QString().setNum(height) + QString(" m")));
    mapOcgd.setText(1,(QString().setNum(ocgd) + QString(" m/px")));
  }
  
  void CInfoLoader::updateRobots(const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
    for(unsigned int i = 0 ; i < msg.robots.size() ; i++)
    {
      QTreeWidgetItem  *rnode = new QTreeWidgetItem();
      rnode->setText(0,QString(msg.robots[i].name.c_str()));
      rnode->setIcon(3,visible_icon_);
      
      QTreeWidgetItem *radius = new QTreeWidgetItem();
      radius->setText(0,"Radius");
      radius->setText(1,(QString().setNum(
        msg.robots[i].robot.footprint.radius) + QString("m")));
      rnode->addChild(radius);
      
      QTreeWidgetItem *lasers = new QTreeWidgetItem(),
                      *sonars = new QTreeWidgetItem(),
                      *rfids = new QTreeWidgetItem(),
                      *kinematics = new QTreeWidgetItem();

      lasers->setText(0,"Lasers");
      sonars->setText(0,"Sonars");
      rfids->setText(0,"RFID antennas");
      kinematics->setText(0,"Kinematic");
      
      for(unsigned int l = 0 ; l < msg.robots[i].robot.laserSensors.size() ; 
          l++)
      {
        QTreeWidgetItem *lname;
        lname=new QTreeWidgetItem();
        lname->setText(0,
          msg.robots[i].robot.laserSensors[l].frame_id.c_str());
        lname->setIcon(3,visible_icon_);

        QTreeWidgetItem *lrays = new QTreeWidgetItem();
        QTreeWidgetItem *lmaxrange = new QTreeWidgetItem();
        QTreeWidgetItem *lminrange = new QTreeWidgetItem();
        QTreeWidgetItem *lmaxangle = new QTreeWidgetItem();
        QTreeWidgetItem *lminangle = new QTreeWidgetItem();
        QTreeWidgetItem *lnoisemean = new QTreeWidgetItem();
        QTreeWidgetItem *lnoisestd = new QTreeWidgetItem();
        QTreeWidgetItem *lfreq = new QTreeWidgetItem();
        QTreeWidgetItem *lposex = new QTreeWidgetItem();
        QTreeWidgetItem *lposey = new QTreeWidgetItem();
        QTreeWidgetItem *lorientation = new QTreeWidgetItem();
        
        lrays->setText(0,"Rays");
        lrays->setText(1,QString().setNum(
          msg.robots[i].robot.laserSensors[l].numRays));
        lmaxrange->setText(0,"Max dist");
        lmaxrange->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].maxRange) + QString(" m")));
        lminrange->setText(0,"Min dist");
        lminrange->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].minRange) + QString(" m")));
        lmaxangle->setText(0,"Max angle");
        lmaxangle->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].maxAngle) + QString(" deg")));
        lminangle->setText(0,"Min angle");
        lminangle->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].minAngle) + QString(" deg")));
        lnoisemean->setText(0,"Noise (mean)");
        lnoisemean->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].noise.noiseMean) + QString(" m")));
        lnoisestd->setText(0,"Noise (std)");
        lnoisestd->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].noise.noiseStd) + QString(" m")));
        lfreq->setText(0,"Frequency");
        lfreq->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].frequency) + QString(" Hz")));
            
        lposex->setText(0,"x pose");
        lposex->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].pose.x) + QString(" m")));
        lposey->setText(0,"y pose");
        lposey->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].pose.y) + QString(" m")));
        lorientation->setText(0,"Orientation");
        lorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].pose.theta) + QString(" rad")));
                  
        lname->addChild(lrays);
        lname->addChild(lmaxrange);
        lname->addChild(lminrange);
        lname->addChild(lmaxangle);
        lname->addChild(lminangle);
        lname->addChild(lnoisemean);
        lname->addChild(lnoisestd);
        lname->addChild(lfreq);
        lname->addChild(lposex);
        lname->addChild(lposey);
        lname->addChild(lorientation);
        
        lasers->addChild(lname);
      }
      
      for(unsigned int l = 0; l < msg.robots[i].robot.sonarSensors.size() ; 
        l++)
      {
        QTreeWidgetItem *sname;
        sname = new QTreeWidgetItem();
        sname->setText(0,
          msg.robots[i].robot.sonarSensors[l].frame_id.c_str());
        sname->setIcon(3,visible_icon_);

        QTreeWidgetItem *smaxrange = new QTreeWidgetItem();
        QTreeWidgetItem *sminrange = new QTreeWidgetItem();
        QTreeWidgetItem *scone = new QTreeWidgetItem();
        QTreeWidgetItem *sorientation = new QTreeWidgetItem();
        QTreeWidgetItem *snoisemean = new QTreeWidgetItem();
        QTreeWidgetItem *snoisestd = new QTreeWidgetItem();
        QTreeWidgetItem *sfreq = new QTreeWidgetItem();
        QTreeWidgetItem *sposex = new QTreeWidgetItem();
        QTreeWidgetItem *sposey = new QTreeWidgetItem();
        
        smaxrange->setText(0,"Max dist");
        smaxrange->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].maxRange) + QString(" m")));
        sminrange->setText(0,"Min dist");
        sminrange->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].minRange) + QString(" m")));
        scone->setText(0,"Cone");
        scone->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].coneAngle) + QString(" deg")));
        sorientation->setText(0,"Orientation");
        sorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].pose.theta) + QString(" deg")));
        snoisemean->setText(0,"Noise (mean)");
        snoisemean->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].noise.noiseMean) + 
            QString(" m")));
        snoisestd->setText(0,"Noise (std)");
        snoisestd->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].noise.noiseStd) + QString(" m")));
        sfreq->setText(0,"Frequency");
        sfreq->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].frequency) + QString(" Hz")));
            
        sposex->setText(0,"x pose");
        sposex->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].pose.x) + QString(" m")));
        sposey->setText(0,"y pose");
        sposey->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].pose.y) + QString(" m")));
                  
        sname->addChild(smaxrange);
        sname->addChild(sminrange);
        sname->addChild(scone);
        sname->addChild(sorientation);
        sname->addChild(snoisemean);
        sname->addChild(snoisestd);
        sname->addChild(sfreq);
        sname->addChild(sposex);
        sname->addChild(sposey);
        
        sonars->addChild(sname);
      }
      
      rnode->addChild(lasers);
      rnode->addChild(sonars);
      rnode->addChild(rfids);
      rnode->addChild(kinematics);
      
      robotsInfo.addChild(rnode);
    }    
  }
}
