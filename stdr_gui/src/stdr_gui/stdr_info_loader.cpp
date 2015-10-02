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
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
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
        
    visible_icon_on_.addFile(QString((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/visible_on.png")).c_str()));
    visible_icon_off_.addFile(QString((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/visible_off.png")).c_str()));
    visible_icon_trans_.addFile(QString((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/visible_transparent.png")).c_str()));
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CInfoLoader::~CInfoLoader(void)
  {
    
  }
  
  /**
  @brief Deletes a specific tree node. Recursive function.
  @param item [QTreeWidgetItem*] The item to be deleted
  @return void
  **/
  void CInfoLoader::deleteTreeNode(QTreeWidgetItem *item)
  {
    int count = item->childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      deleteTreeNode(item->child(i));
      QTreeWidgetItem *child = item->child(i);
      item->removeChild(item->child(i));
      delete child;
    }
  }
  
  /**
  @brief Deletes the information tree
  @return void
  **/
  void CInfoLoader::deleteTree(void)
  {
    int count = robotsInfo.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      //~ deleteTreeNode(robotsInfo.child(i));
      QTreeWidgetItem *child = robotsInfo.child(i);
      robotsInfo.removeChild(robotsInfo.child(i));
      //~ delete child;
    }
  }
  
  /**
  @brief Updates the information tree according to the specific map
  @param width [float] The map width
  @param height [float] The map height
  @param ocgd [float] The map resolution (m/pixel)
  @return void
  **/
  void CInfoLoader::updateMapInfo(float width,float height,float ocgd)
  {
    mapWidth.setText(1,(QString().setNum(width) + QString(" m")));
    mapHeight.setText(1,(QString().setNum(height) + QString(" m")));
    mapOcgd.setText(1,(QString().setNum(ocgd) + QString(" m/px")));
  }
  
  /**
  @brief Updates the information tree according to the ensemble of robots
  @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The existent robots
  @return void
  **/
  void CInfoLoader::updateRobots(const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
    for(unsigned int i = 0 ; i < msg.robots.size() ; i++)
    {
      QTreeWidgetItem  *rnode = new QTreeWidgetItem();
      rnode->setText(0,QString(msg.robots[i].name.c_str()));
      rnode->setIcon(2,visible_icon_on_);
      rnode->setIcon(3,visible_icon_);
      rnode->setToolTip(2,"Visibility status");
      rnode->setToolTip(3,"Visualize robot topics");
      
      QTreeWidgetItem *radius = new QTreeWidgetItem();
      radius->setText(0,"Radius");
      radius->setText(1,(QString().setNum(
        msg.robots[i].robot.footprint.radius) + QString("m")));
      rnode->addChild(radius);
      
      QTreeWidgetItem *lasers = new QTreeWidgetItem(),
                      *sonars = new QTreeWidgetItem(),
                      *rfids = new QTreeWidgetItem(),
                      *co2_sensors = new QTreeWidgetItem(),
                      *thermal_sensors = new QTreeWidgetItem(),
                      *sound_sensors = new QTreeWidgetItem(),
                      *kinematics = new QTreeWidgetItem();

      lasers->setText(0,"Lasers");
      sonars->setText(0,"Sonars");
      rfids->setText(0,"RFID readers");
      co2_sensors->setText(0,"CO2 sensors");
      thermal_sensors->setText(0,"Thermal sensors");
      sound_sensors->setText(0,"Sound sensors");
      kinematics->setText(0,"Kinematic");
      kinematics->setText(1,
        QString(msg.robots[i].robot.kinematicModel.type.c_str()));
      
      for(unsigned int l = 0 ; l < msg.robots[i].robot.laserSensors.size() ; 
          l++)
      {
        QTreeWidgetItem *lname;
        lname=new QTreeWidgetItem();
        lname->setText(0,
          msg.robots[i].robot.laserSensors[l].frame_id.c_str());
          
        lname->setIcon(2,visible_icon_on_);
        lname->setToolTip(2,"Visibility status");
        lname->setIcon(3,visible_icon_);
        lname->setToolTip(3,"Visualize topic");

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
          msg.robots[i].robot.laserSensors[l].maxAngle) + QString(" rad")));
        lminangle->setText(0,"Min angle");
        lminangle->setText(1,(QString().setNum(
          msg.robots[i].robot.laserSensors[l].minAngle) + QString(" rad")));
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
        sname->setIcon(2,visible_icon_on_);
        sname->setIcon(3,visible_icon_);
        
        sname->setToolTip(2,"Visibility status");
        sname->setToolTip(3,"Visualize topic");

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
          msg.robots[i].robot.sonarSensors[l].coneAngle) + QString(" rad")));
        sorientation->setText(0,"Orientation");
        sorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.sonarSensors[l].pose.theta) + QString(" rad")));
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
      
      for(unsigned int l = 0; l < msg.robots[i].robot.rfidSensors.size() ; 
        l++)
      {
        QTreeWidgetItem *sname;
        sname = new QTreeWidgetItem();
        sname->setText(0,
          msg.robots[i].robot.rfidSensors[l].frame_id.c_str());
        sname->setIcon(2,visible_icon_on_);
        //~ sname->setIcon(3,visible_icon_);
        
        sname->setToolTip(2,"Visibility status");
        //~ sname->setToolTip(3,"Visualize topic");

        QTreeWidgetItem *smaxrange = new QTreeWidgetItem();
        QTreeWidgetItem *sspan = new QTreeWidgetItem();
        QTreeWidgetItem *sorientation = new QTreeWidgetItem();
        QTreeWidgetItem *sfreq = new QTreeWidgetItem();
        QTreeWidgetItem *scutoff = new QTreeWidgetItem();
        QTreeWidgetItem *sposex = new QTreeWidgetItem();
        QTreeWidgetItem *sposey = new QTreeWidgetItem();
        
        smaxrange->setText(0,"Max dist");
        smaxrange->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].maxRange) + QString(" m")));
       
        sspan->setText(0,"Angle span");
        sspan->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].angleSpan) 
            + QString(" rad")));
          
        sorientation->setText(0,"Orientation");
        sorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].pose.theta) + 
            QString(" rad")));
          
        scutoff->setText(0,"Signal Cutoff");
        scutoff->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].signalCutoff) + 
            QString("")));

        sfreq->setText(0,"Frequency");
        sfreq->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].frequency) + QString(" Hz")));
            
        sposex->setText(0,"x pose");
        sposex->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].pose.x) + QString(" m")));
          
        sposey->setText(0,"y pose");
        sposey->setText(1,(QString().setNum(
          msg.robots[i].robot.rfidSensors[l].pose.y) + QString(" m")));
                  
        sname->addChild(smaxrange);
        sname->addChild(sspan);
        sname->addChild(sorientation);
        sname->addChild(scutoff);
        sname->addChild(sfreq);
        sname->addChild(sposex);
        sname->addChild(sposey);
        
        rfids->addChild(sname);
      }
      for(unsigned int l = 0; l < msg.robots[i].robot.co2Sensors.size() ; 
        l++)
      {
        QTreeWidgetItem *sname;
        sname = new QTreeWidgetItem();
        sname->setText(0,
          msg.robots[i].robot.co2Sensors[l].frame_id.c_str());
        sname->setIcon(2,visible_icon_on_);
        //~ sname->setIcon(3,visible_icon_);
        
        sname->setToolTip(2,"Visibility status");
        //~ sname->setToolTip(3,"Visualize topic");

        QTreeWidgetItem *smaxrange = new QTreeWidgetItem();
        //~ QTreeWidgetItem *sspan = new QTreeWidgetItem();
        QTreeWidgetItem *sorientation = new QTreeWidgetItem();
        QTreeWidgetItem *sfreq = new QTreeWidgetItem();
        //~ QTreeWidgetItem *scutoff = new QTreeWidgetItem();
        QTreeWidgetItem *sposex = new QTreeWidgetItem();
        QTreeWidgetItem *sposey = new QTreeWidgetItem();
        
        smaxrange->setText(0,"Max dist");
        smaxrange->setText(1,(QString().setNum(
          msg.robots[i].robot.co2Sensors[l].maxRange) + QString(" m")));
       
        //~ sspan->setText(0,"Angle span");
        //~ sspan->setText(1,(QString().setNum(
          //~ msg.robots[i].robot.rfidSensors[l].angleSpan) 
            //~ + QString(" rad")));
          
        sorientation->setText(0,"Orientation");
        sorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.co2Sensors[l].pose.theta) + 
            QString(" rad")));
          
        //~ scutoff->setText(0,"Signal Cutoff");
        //~ scutoff->setText(1,(QString().setNum(
          //~ msg.robots[i].robot.rfidSensors[l].signalCutoff) + 
            //~ QString("")));

        sfreq->setText(0,"Frequency");
        sfreq->setText(1,(QString().setNum(
          msg.robots[i].robot.co2Sensors[l].frequency) + QString(" Hz")));
            
        sposex->setText(0,"x pose");
        sposex->setText(1,(QString().setNum(
          msg.robots[i].robot.co2Sensors[l].pose.x) + QString(" m")));
          
        sposey->setText(0,"y pose");
        sposey->setText(1,(QString().setNum(
          msg.robots[i].robot.co2Sensors[l].pose.y) + QString(" m")));
                  
        sname->addChild(smaxrange);
        //~ sname->addChild(sspan);
        sname->addChild(sorientation);
        //~ sname->addChild(scutoff);
        sname->addChild(sfreq);
        sname->addChild(sposex);
        sname->addChild(sposey);
        
        co2_sensors->addChild(sname);
      }
      for(unsigned int l = 0; l < msg.robots[i].robot.thermalSensors.size() ; 
        l++)
      {
        QTreeWidgetItem *sname;
        sname = new QTreeWidgetItem();
        sname->setText(0,
          msg.robots[i].robot.thermalSensors[l].frame_id.c_str());
        sname->setIcon(2,visible_icon_on_);
        //~ sname->setIcon(3,visible_icon_);
        
        sname->setToolTip(2,"Visibility status");
        //~ sname->setToolTip(3,"Visualize topic");

        QTreeWidgetItem *smaxrange = new QTreeWidgetItem();
        QTreeWidgetItem *sspan = new QTreeWidgetItem();
        QTreeWidgetItem *sorientation = new QTreeWidgetItem();
        QTreeWidgetItem *sfreq = new QTreeWidgetItem();
        //~ QTreeWidgetItem *scutoff = new QTreeWidgetItem();
        QTreeWidgetItem *sposex = new QTreeWidgetItem();
        QTreeWidgetItem *sposey = new QTreeWidgetItem();
        
        smaxrange->setText(0,"Max dist");
        smaxrange->setText(1,(QString().setNum(
          msg.robots[i].robot.thermalSensors[l].maxRange) + QString(" m")));
       
        sspan->setText(0,"Angle span");
        sspan->setText(1,(QString().setNum(
          msg.robots[i].robot.thermalSensors[l].angleSpan) 
            + QString(" rad")));
          
        sorientation->setText(0,"Orientation");
        sorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.thermalSensors[l].pose.theta) + 
            QString(" rad")));
          
        //~ scutoff->setText(0,"Signal Cutoff");
        //~ scutoff->setText(1,(QString().setNum(
          //~ msg.robots[i].robot.rfidSensors[l].signalCutoff) + 
            //~ QString("")));

        sfreq->setText(0,"Frequency");
        sfreq->setText(1,(QString().setNum(
          msg.robots[i].robot.thermalSensors[l].frequency) + QString(" Hz")));
            
        sposex->setText(0,"x pose");
        sposex->setText(1,(QString().setNum(
          msg.robots[i].robot.thermalSensors[l].pose.x) + QString(" m")));
          
        sposey->setText(0,"y pose");
        sposey->setText(1,(QString().setNum(
          msg.robots[i].robot.thermalSensors[l].pose.y) + QString(" m")));
                  
        sname->addChild(smaxrange);
        sname->addChild(sspan);
        sname->addChild(sorientation);
        //~ sname->addChild(scutoff);
        sname->addChild(sfreq);
        sname->addChild(sposex);
        sname->addChild(sposey);
        
        thermal_sensors->addChild(sname);
      }
      for(unsigned int l = 0; l < msg.robots[i].robot.soundSensors.size() ; 
        l++)
      {
        QTreeWidgetItem *sname;
        sname = new QTreeWidgetItem();
        sname->setText(0,
          msg.robots[i].robot.soundSensors[l].frame_id.c_str());
        sname->setIcon(2,visible_icon_on_);
        //~ sname->setIcon(3,visible_icon_);
        
        sname->setToolTip(2,"Visibility status");
        //~ sname->setToolTip(3,"Visualize topic");

        QTreeWidgetItem *smaxrange = new QTreeWidgetItem();
        QTreeWidgetItem *sspan = new QTreeWidgetItem();
        QTreeWidgetItem *sorientation = new QTreeWidgetItem();
        QTreeWidgetItem *sfreq = new QTreeWidgetItem();
        //~ QTreeWidgetItem *scutoff = new QTreeWidgetItem();
        QTreeWidgetItem *sposex = new QTreeWidgetItem();
        QTreeWidgetItem *sposey = new QTreeWidgetItem();
        
        smaxrange->setText(0,"Max dist");
        smaxrange->setText(1,(QString().setNum(
          msg.robots[i].robot.soundSensors[l].maxRange) + QString(" m")));
       
        sspan->setText(0,"Angle span");
        sspan->setText(1,(QString().setNum(
          msg.robots[i].robot.soundSensors[l].angleSpan) 
            + QString(" rad")));
          
        sorientation->setText(0,"Orientation");
        sorientation->setText(1,(QString().setNum(
          msg.robots[i].robot.soundSensors[l].pose.theta) + 
            QString(" rad")));
          
        //~ scutoff->setText(0,"Signal Cutoff");
        //~ scutoff->setText(1,(QString().setNum(
          //~ msg.robots[i].robot.rfidSensors[l].signalCutoff) + 
            //~ QString("")));

        sfreq->setText(0,"Frequency");
        sfreq->setText(1,(QString().setNum(
          msg.robots[i].robot.soundSensors[l].frequency) + QString(" Hz")));
            
        sposex->setText(0,"x pose");
        sposex->setText(1,(QString().setNum(
          msg.robots[i].robot.soundSensors[l].pose.x) + QString(" m")));
          
        sposey->setText(0,"y pose");
        sposey->setText(1,(QString().setNum(
          msg.robots[i].robot.soundSensors[l].pose.y) + QString(" m")));
                  
        sname->addChild(smaxrange);
        sname->addChild(sspan);
        sname->addChild(sorientation);
        //~ sname->addChild(scutoff);
        sname->addChild(sfreq);
        sname->addChild(sposex);
        sname->addChild(sposey);
        
        sound_sensors->addChild(sname);
      }
      
      rnode->addChild(lasers);
      rnode->addChild(sonars);
      rnode->addChild(rfids);
      rnode->addChild(co2_sensors);
      rnode->addChild(thermal_sensors);
      rnode->addChild(sound_sensors);
      rnode->addChild(kinematics);
      
      robotsInfo.addChild(rnode);
    }       
  }
}
