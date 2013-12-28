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

#include "stdr_gui/stdr_robot_creator/stdr_robot_creator_connector.h"

namespace stdr_gui
{
  
  unsigned int CRobotCreatorConnector::laser_number = 0;
  unsigned int CRobotCreatorConnector::sonar_number = 0;
  unsigned int CRobotCreatorConnector::rfid_number = 0;
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char**] Input arguments
  @return void
  **/
  CRobotCreatorConnector::CRobotCreatorConnector(int argc, char **argv):
    QObject(),
    loader_(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    QObject::connect(
      loader_.robotTreeWidget,SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this,SLOT(treeItemClicked(QTreeWidgetItem *, int)));
      
    QObject::connect(
      loader_.laserPropLoader.laserUpdateButton,SIGNAL(clicked(bool)),
      this,SLOT(updateLaser()));
    
    QObject::connect(
      loader_.robotPropLoader.updateButton,SIGNAL(clicked(bool)),
      this,SLOT(updateRobot()));
    
    QObject::connect(
      loader_.sonarPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateSonar()));
    
    QObject::connect(
      loader_.rfidAntennaPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateRfid()));
    
    QObject::connect(
      loader_.loadRobotButton,SIGNAL(clicked(bool)),
      this,SLOT(loadRobot()));

    climax_ = - 1;
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CRobotCreatorConnector::~CRobotCreatorConnector(void)
  {
    
  }
  
  /**
  @brief Initializes the robot creator
  @return void
  **/
  void CRobotCreatorConnector::initialise(void)
  {
    new_robot_msg_ = stdr_msgs::RobotMsg();
    
    loader_.robotInfoShape.setText(0,"Shape");
    loader_.robotInfoShape.setText(1,"Circle");
    loader_.robotInfoOrientation.setText(0,"Orientation");
    loader_.robotInfoOrientation.setText(1,"0");
    
    new_robot_msg_.footprint.radius = 0.15;
    
    unsigned int laserCount = loader_.lasersNode.childCount();
    unsigned int sonarCount = loader_.sonarsNode.childCount();
    unsigned int rfidCount = loader_.rfidAntennasNode.childCount();
    
    for(int i = laserCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.lasersNode.child(i));
    for(int i = sonarCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.sonarsNode.child(i));
    for(int i = rfidCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.rfidAntennasNode.child(i));
    
    CRobotCreatorConnector::laser_number = 0;
    CRobotCreatorConnector::sonar_number = 0;
    CRobotCreatorConnector::rfid_number = 0;
    
    updateRobotPreview();
    
    loader_.show();
  }

  /**
  @brief Called when a tree item is clicked
  @param item [QTreeWidgetItem *] The item clicked
  @param column [int] The column clicked
  @return void
  **/
  void CRobotCreatorConnector::treeItemClicked( 
    QTreeWidgetItem * item, 
    int column)
  {
    //!< Robot edit clicked
    if(item == &loader_.robotNode && column == 2)
    {    
      editRobot();
    }  
    //!< Robot save clicked
    if(item == &loader_.robotNode && column == 3)
    {    
      saveRobot();
    }  
    //!< Robot load clicked
    if(item == &loader_.robotNode && column == 4)
    {    
      getRobotFromYaml();
    }  
    //!< Kinematic edit clicked  
    if(item == &loader_.kinematicNode && column == 2)
    {  
      loader_.kinematicPropLoader.show();
    }
    //!< Add laser clicked
    if(item == &loader_.lasersNode && column == 2)
    {
      addLaser();
    }  
    //!< Erase a laser
    if(item->parent() == &loader_.lasersNode && column == 2)
    {
      eraseLaser(item);
    }  
    //!< Edit a laser
    if(item->parent() == &loader_.lasersNode && column == 1)
    {
      editLaser(item);
    }  
    //!< Add a sonar
    if(item == &loader_.sonarsNode && column == 2)
    {
      addSonar();
    }  
    //!< Add a rfid antenna
    if(item == &loader_.rfidAntennasNode && column == 2)
    {
      //~ addRfidAntenna();
    }  
    //!< Erase a sonar
    if(item->parent() == &loader_.sonarsNode && column == 2)
    {
      eraseSonar(item);
    }  
    //!< Erase a rfid antenna
    if(item->parent() == &loader_.rfidAntennasNode && column == 2)
    {
      //~ eraseRfid(item);
    }  
    //!< Edit a sonar
    if(item->parent() == &loader_.sonarsNode && column == 1)
    {
      editSonar(item);
    }  
    //!< Edit a rfid antenna  
    if(item->parent() == &loader_.rfidAntennasNode && column == 1)
    {
      //~ editRfid(item);
    }
    //!< Save a laser  
    if(item->parent() == &loader_.lasersNode && column == 3)
    {
      saveLaser(item);
    }
    //!< Load a laser  
    if(item->parent() == &loader_.lasersNode && column == 4)
    {
      loadLaser(item);
    }
    //!< Save a sonar  
    if(item->parent() == &loader_.sonarsNode && column == 3)
    {
      saveSonar(item);
    }
    //!< Load a sonar  
    if(item->parent() == &loader_.sonarsNode && column == 4)
    {
      loadSonar(item);
    }
  }

  /**
  @brief Adds a laser sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addLaser(void)
  {
    QString laserFrameId=QString("laser_") + 
      QString().setNum(CRobotCreatorConnector::laser_number++);
    
    stdr_msgs::LaserSensorMsg lmsg;
    lmsg.frame_id = laserFrameId.toStdString();
    lmsg.numRays = 270;
    lmsg.maxAngle = 135;
    lmsg.minAngle = - 135;
    lmsg.maxRange = 4.0;
    lmsg.minRange = 0.0;
    lmsg.pose.x = 0;
    lmsg.pose.y = 0;
    lmsg.pose.theta = 0;
    lmsg.noise.noiseMean = 0;
    lmsg.noise.noiseStd = 0;
    lmsg.frequency = 10.0;
    
    new_robot_msg_.laserSensors.push_back(lmsg);
    
    QTreeWidgetItem  *lnode;
    lnode = new QTreeWidgetItem();
    lnode->setText(0,laserFrameId);
    lnode->setIcon(1,loader_.editIcon);
    lnode->setIcon(2,loader_.removeIcon);
    lnode->setIcon(3,loader_.saveIcon);
    lnode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      *angleSpan,
      *orientation,
      *maxRange,
      *minRange,
      *noiseMean,
      *noiseStd,
      *poseX,
      *poseY,
      *frequency,
      *rays;
      
    rays = new QTreeWidgetItem();
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    minRange = new QTreeWidgetItem();
    noiseMean = new QTreeWidgetItem();
    noiseStd = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    rays->setText(0,QString("Number of rays"));
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    minRange->setText(0,QString("Min range"));
    noiseMean->setText(0,QString("Noise mean"));
    noiseStd->setText(0,QString("Noise std"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    frequency->setText(0,QString("Frequency"));
    
    rays->setText(1,QString().setNum(lmsg.numRays));
    angleSpan->setText(1,QString().setNum(lmsg.maxAngle - lmsg.minAngle));
    orientation->setText(1,QString().setNum(lmsg.maxAngle + lmsg.minAngle));
    maxRange->setText(1,QString().setNum(lmsg.maxRange));
    minRange->setText(1,QString().setNum(lmsg.minRange));
    noiseMean->setText(1,QString().setNum(lmsg.noise.noiseMean));
    noiseStd->setText(1,QString().setNum(lmsg.noise.noiseStd));
    poseX->setText(1,QString().setNum(lmsg.pose.x));
    poseY->setText(1,QString().setNum(lmsg.pose.y));
    frequency->setText(1,QString().setNum(lmsg.frequency));
    
    lnode->addChild(rays);
    lnode->addChild(angleSpan);
    lnode->addChild(orientation);
    lnode->addChild(maxRange);
    lnode->addChild(minRange);
    lnode->addChild(noiseMean);
    lnode->addChild(noiseStd);
    lnode->addChild(poseX);
    lnode->addChild(poseY);
    lnode->addChild(frequency);
    
    loader_.lasersNode.addChild(lnode);
    
    lnode->setExpanded(false);
    loader_.lasersNode.setExpanded(true);
    updateRobotPreview();
  }
  
  /**
  @brief Adds a specific laser sensor in the new robot 
  @param lmsg [stdr_msgs::LaserSensorMsg] The laser sensor to be added
  @return void
  **/
  void CRobotCreatorConnector::addLaser(stdr_msgs::LaserSensorMsg lmsg)
  {
    CRobotCreatorConnector::laser_number++;
    QString laserFrameId=QString(lmsg.frame_id.c_str());
    
    QTreeWidgetItem  *lnode;
    lnode = new QTreeWidgetItem();
    lnode->setText(0,laserFrameId);
    lnode->setIcon(1,loader_.editIcon);
    lnode->setIcon(2,loader_.removeIcon);
    lnode->setIcon(3,loader_.saveIcon);
    lnode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      *angleSpan,
      *orientation,
      *maxRange,
      *minRange,
      *noiseMean,
      *noiseStd,
      *poseX,
      *poseY,
      *frequency,
      *rays;
      
    rays = new QTreeWidgetItem();
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    minRange = new QTreeWidgetItem();
    noiseMean = new QTreeWidgetItem();
    noiseStd = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    rays->setText(0,QString("Number of rays"));
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    minRange->setText(0,QString("Min range"));
    noiseMean->setText(0,QString("Noise mean"));
    noiseStd->setText(0,QString("Noise std"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    frequency->setText(0,QString("Frequency"));
    
    rays->setText(1,QString().setNum(lmsg.numRays));
    angleSpan->setText(1,QString().setNum(lmsg.maxAngle - lmsg.minAngle));
    orientation->setText(1,QString().setNum(lmsg.maxAngle + lmsg.minAngle));
    maxRange->setText(1,QString().setNum(lmsg.maxRange));
    minRange->setText(1,QString().setNum(lmsg.minRange));
    noiseMean->setText(1,QString().setNum(lmsg.noise.noiseMean));
    noiseStd->setText(1,QString().setNum(lmsg.noise.noiseStd));
    poseX->setText(1,QString().setNum(lmsg.pose.x));
    poseY->setText(1,QString().setNum(lmsg.pose.y));
    frequency->setText(1,QString().setNum(lmsg.frequency));
    
    lnode->addChild(rays);
    lnode->addChild(angleSpan);
    lnode->addChild(orientation);
    lnode->addChild(maxRange);
    lnode->addChild(minRange);
    lnode->addChild(noiseMean);
    lnode->addChild(noiseStd);
    lnode->addChild(poseX);
    lnode->addChild(poseY);
    lnode->addChild(frequency);
    
    loader_.lasersNode.addChild(lnode);
    
    lnode->setExpanded(false);
    loader_.lasersNode.setExpanded(true);
  }
  
  /**
  @brief Adds a sonar sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addSonar(void)
  {
    QString sonarFrameId = 
      QString("sonar_") + 
      QString().setNum(CRobotCreatorConnector::sonar_number++);
    
    stdr_msgs::SonarSensorMsg smsg;
    smsg.frame_id = sonarFrameId.toStdString();
    smsg.maxRange = 3.0;
    smsg.minRange = 0.3;
    smsg.coneAngle = 50.0;
    smsg.pose.x = 0;
    smsg.pose.y = 0;
    smsg.pose.theta = 0;
    smsg.noise.noiseMean = 0;
    smsg.noise.noiseStd = 0;
    smsg.frequency = 10;
    
    new_robot_msg_.sonarSensors.push_back(smsg);
    
    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,sonarFrameId);
    snode->setIcon(1,loader_.editIcon);
    snode->setIcon(2,loader_.removeIcon);
    snode->setIcon(3,loader_.saveIcon);
    snode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      *coneAngle,
      *orientation,
      *maxRange,
      *minRange,
      *noiseMean,
      *noiseStd,
      *poseX,
      *poseY,
      *frequency;
      
    coneAngle = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    minRange = new QTreeWidgetItem();
    noiseMean = new QTreeWidgetItem();
    noiseStd = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    coneAngle->setText(0,QString("Cone span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    minRange->setText(0,QString("Min range"));
    noiseMean->setText(0,QString("Noise mean"));
    noiseStd->setText(0,QString("Noise std"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    frequency->setText(0,QString("Frequency"));
    
    coneAngle->setText(1,QString().setNum(smsg.coneAngle));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    minRange->setText(1,QString().setNum(smsg.minRange));
    noiseMean->setText(1,QString().setNum(smsg.noise.noiseMean));
    noiseStd->setText(1,QString().setNum(smsg.noise.noiseStd));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(coneAngle);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(minRange);
    snode->addChild(noiseMean);
    snode->addChild(noiseStd);
    snode->addChild(poseX);
    snode->addChild(poseY);
    snode->addChild(frequency);
    
    loader_.sonarsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.sonarsNode.setExpanded(true);
    updateRobotPreview();
  }
  
  /**
  @brief Adds a specific sonar sensor in the new robot 
  @param smsg [stdr_msgs::SonarSensorMsg] The sonar sensor to be added
  @return void
  **/
  void CRobotCreatorConnector::addSonar(stdr_msgs::SonarSensorMsg smsg)
  {
    CRobotCreatorConnector::sonar_number++;
    QString sonarFrameId = QString(smsg.frame_id.c_str());

    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,sonarFrameId);
    snode->setIcon(1,loader_.editIcon);
    snode->setIcon(2,loader_.removeIcon);
    snode->setIcon(3,loader_.saveIcon);
    snode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      *coneAngle,
      *orientation,
      *maxRange,
      *minRange,
      *noiseMean,
      *noiseStd,
      *poseX,
      *poseY,
      *frequency;
      
    coneAngle = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    minRange = new QTreeWidgetItem();
    noiseMean = new QTreeWidgetItem();
    noiseStd = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    coneAngle->setText(0,QString("Cone span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    minRange->setText(0,QString("Min range"));
    noiseMean->setText(0,QString("Noise mean"));
    noiseStd->setText(0,QString("Noise std"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    frequency->setText(0,QString("Frequency"));
    
    coneAngle->setText(1,QString().setNum(smsg.coneAngle));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    minRange->setText(1,QString().setNum(smsg.minRange));
    noiseMean->setText(1,QString().setNum(smsg.noise.noiseMean));
    noiseStd->setText(1,QString().setNum(smsg.noise.noiseStd));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(coneAngle);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(minRange);
    snode->addChild(noiseMean);
    snode->addChild(noiseStd);
    snode->addChild(poseX);
    snode->addChild(poseY);
    snode->addChild(frequency);
    
    loader_.sonarsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.sonarsNode.setExpanded(true);
  }
  
  /**
  @brief Adds an rfid antenna sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addRfidAntenna(void)
  {
    QString rfidFrameId=QString("rfid_antenna_") + 
      QString().setNum(CRobotCreatorConnector::rfid_number++);
    
    stdr_msgs::RfidSensorMsg smsg;
    smsg.frame_id = rfidFrameId.toStdString();
    smsg.maxRange = 3.0;
    smsg.angleSpan = 360.0;
    smsg.pose.x = 0;
    smsg.pose.y = 0;
    smsg.pose.theta = 0;
    smsg.signalCutoff = 0;
    smsg.frequency = 10;
    
    new_robot_msg_.rfidSensors.push_back(smsg);
    
    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,rfidFrameId);
    snode->setIcon(1,loader_.editIcon);
    snode->setIcon(2,loader_.removeIcon);
    snode->setIcon(3,loader_.saveIcon);
    snode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      *angleSpan,
      *orientation,
      *maxRange,
      *poseX,
      *poseY,
      *signalCutoff,
      *frequency;
      
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.rfidAntennasNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.rfidAntennasNode.setExpanded(true);
    updateRobotPreview();
  }
  
  /**
  @brief Erases a specific laser sensor based on a tree item
  @param item [QTreeWidgetItem*] Tree item that holds the specific laser sensor 
  @return void
  **/
  void CRobotCreatorConnector::eraseLaser(QTreeWidgetItem *item)
  {
    unsigned int laserFrameId = searchLaser(item->text(0));
    if(laserFrameId == -1) 
    {
      return;
    }
    new_robot_msg_.laserSensors.erase(
      new_robot_msg_.laserSensors.begin() + laserFrameId);
    deleteTreeNode(item);
    updateRobotPreview();
  }
  
  /**
  @brief Erases a specific sonar sensor based on a tree item
  @param item [QTreeWidgetItem*] Tree item that holds the specific sonar sensor 
  @return void
  **/
  void CRobotCreatorConnector::eraseSonar(QTreeWidgetItem *item)
  {
    unsigned int sonarFrameId = searchSonar(item->text(0));
    if(sonarFrameId == -1) 
    {
      return;
    }
    new_robot_msg_.sonarSensors.erase(
      new_robot_msg_.sonarSensors.begin() + sonarFrameId);
    deleteTreeNode(item);
    updateRobotPreview();
  }
  
  /**
  @brief Erases a specific rfid antenna sensor based on a tree item
  @param item [QTreeWidgetItem*] Tree item that holds the specific rfid antenna sensor 
  @return void
  **/
  void CRobotCreatorConnector::eraseRfid(QTreeWidgetItem *item)
  {
    unsigned int rfidFrameId = searchRfid(item->text(0));
    if(rfidFrameId == -1) 
    {
      return;
    }
    new_robot_msg_.rfidSensors.erase(
      new_robot_msg_.rfidSensors.begin() + rfidFrameId);
    deleteTreeNode(item);
    updateRobotPreview();
  }
  
  /**
  @brief Edits a specific laser sensor based on a tree item. Initiates the laser sensor editor widget
  @param item [QTreeWidgetItem*] Tree item that holds the specific laser sensor 
  @return void
  **/
  void CRobotCreatorConnector::editLaser(QTreeWidgetItem *item)
  {
    unsigned int laserFrameId = searchLaser(item->text(0));
    if(laserFrameId == -1) 
    {
      return;
    }  
    loader_.laserPropLoader.laserRays->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].numRays));
        
    loader_.laserPropLoader.laserMaxDistance->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].maxRange));
    
    loader_.laserPropLoader.laserMinDistance->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].minRange));
    
    loader_.laserPropLoader.laserAngleSpan->setText(
      QString().setNum(
        new_robot_msg_.laserSensors[laserFrameId].maxAngle - 
        new_robot_msg_.laserSensors[laserFrameId].minAngle));
    
    loader_.laserPropLoader.laserOrientation->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].pose.theta));
    
    loader_.laserPropLoader.laserNoiseMean->setText(
      QString().setNum(
        new_robot_msg_.laserSensors[laserFrameId].noise.noiseMean));
    
    loader_.laserPropLoader.laserNoiseStd->setText(
      QString().setNum(
        new_robot_msg_.laserSensors[laserFrameId].noise.noiseStd));
    
    loader_.laserPropLoader.laserTranslationX->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].pose.x));
    
    loader_.laserPropLoader.laserTranslationY->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].pose.y));
    
    loader_.laserPropLoader.laserFrequency->setText(
      QString().setNum(new_robot_msg_.laserSensors[laserFrameId].frequency));
    
    loader_.laserPropLoader.setWindowTitle(
      QApplication::translate(
        "LaserProperties", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_laser_ = item;
    
    loader_.laserPropLoader.show();
  }
  
  /**
  @brief Saves a specific laser sensor in a file
  @param item [QTreeWidgetItem*] The tree item that holds the laser sensor 
  @return void
  **/
  void CRobotCreatorConnector::saveLaser(QTreeWidgetItem *item)
  {
    unsigned int laserFrameId = searchLaser(item->text(0));
    if(laserFrameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save laser sensor"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Yaml files (*.yaml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::LaserSensorMsg lmsg = new_robot_msg_.laserSensors[laserFrameId];
    try {
      stdr_robot::parser::laserSensorMsgToYaml(file_name_str,
        stdr_gui_tools::fixLaserAnglesToRad(lmsg));
    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }
  
  /**
  @brief Saves a specific sonar sensor in a file
  @param item [QTreeWidgetItem*] The tree item that holds the sonar sensor 
  @return void
  **/
  void CRobotCreatorConnector::saveSonar(QTreeWidgetItem *item)
  {
    unsigned int sonarFrameId = searchSonar(item->text(0));
    if(sonarFrameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save sonar sensor"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Yaml files (*.yaml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::SonarSensorMsg smsg = new_robot_msg_.sonarSensors[sonarFrameId];
    try {
      stdr_robot::parser::sonarSensorMsgToYaml(file_name_str,
        stdr_gui_tools::fixSonarAnglesToRad(smsg));
    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }
  
  /**
  @brief Loads a specific laser sensor from a file
  @param item [QTreeWidgetItem*] The tree item that holds the laser sensor 
  @return void
  **/
  void CRobotCreatorConnector::loadLaser(QTreeWidgetItem *item)
  {
    unsigned int laserFrameId = searchLaser(item->text(0));
    if(laserFrameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load laser sensor"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Yaml Files (*.yaml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    try {
      std::string old_frame_id = item->text(0).toStdString();
      
      stdr_msgs::LaserSensorMsg lmsg = 
      stdr_robot::parser::yamlToLaserSensorMsg(file_name.toStdString());

      lmsg = stdr_gui_tools::fixLaserAnglesToDegrees(lmsg);
      lmsg.frame_id = old_frame_id;
      new_robot_msg_.laserSensors[laserFrameId]=lmsg;
      updateLaserTree(item,lmsg);
    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
    updateRobotPreview(); 
  }
  
  /**
  @brief Loads a specific sonar sensor from a file
  @param item [QTreeWidgetItem*] The tree item that holds the sonar sensor 
  @return void
  **/
  void CRobotCreatorConnector::loadSonar(QTreeWidgetItem *item)
  {
    unsigned int sonarFrameId = searchSonar(item->text(0));
    if(sonarFrameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load sonar sensor"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Yaml Files (*.yaml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    try {
      std::string old_frame_id = item->text(0).toStdString();
      
      stdr_msgs::SonarSensorMsg smsg = 
      stdr_robot::parser::yamlToSonarSensorMsg(file_name.toStdString());

      smsg = stdr_gui_tools::fixSonarAnglesToDegrees(smsg);
      
      smsg.frame_id = old_frame_id;
      
      new_robot_msg_.sonarSensors[sonarFrameId]=smsg;
      updateSonarTree(item,smsg);
    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
    updateRobotPreview(); 
  }
  
  /**
  @brief Edits a specific sonar sensor based on a tree item. Initiates the sonar sensor editor widget
  @param item [QTreeWidgetItem*] Tree item that holds the specific sonar sensor 
  @return void
  **/
  void CRobotCreatorConnector::editSonar(QTreeWidgetItem *item)
  {
    unsigned int sonarFrameId = searchSonar(item->text(0));
    if(sonarFrameId == -1) 
    {
      return;
    }  
    loader_.sonarPropLoader.sonarMaxDistance->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].maxRange));
    
    loader_.sonarPropLoader.sonarMinDistance->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].minRange));
    
    loader_.sonarPropLoader.sonarX->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].pose.x));
    
    loader_.sonarPropLoader.sonarY->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].pose.y));
    
    loader_.sonarPropLoader.sonarConeSpan->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].coneAngle));
    
    loader_.sonarPropLoader.sonarNoiseMean->setText(
      QString().setNum(
        new_robot_msg_.sonarSensors[sonarFrameId].noise.noiseMean));
    
    loader_.sonarPropLoader.sonarNoiseStd->setText(
      QString().setNum(
        new_robot_msg_.sonarSensors[sonarFrameId].noise.noiseStd));
    
    loader_.sonarPropLoader.sonarOrientation->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].pose.theta));
    
    loader_.sonarPropLoader.sonarFrequency->setText(
      QString().setNum(new_robot_msg_.sonarSensors[sonarFrameId].frequency));
    
    loader_.sonarPropLoader.setWindowTitle(
      QApplication::translate(
        "SonarProperties", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_sonar_ = item;
    
    loader_.sonarPropLoader.show();
  }
  
  /**
  @brief Edits a specific rfid antenna sensor based on a tree item. Initiates the rfid antenna sensor editor widget
  @param item [QTreeWidgetItem*] Tree item that holds the specific rfid antenna sensor 
  @return void
  **/
  void CRobotCreatorConnector::editRfid(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchRfid(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    loader_.rfidAntennaPropLoader.rfidMaxDistance->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].maxRange));
  
    loader_.rfidAntennaPropLoader.rfidX->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].pose.x));
  
    loader_.rfidAntennaPropLoader.rfidY->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].pose.y));
  
    loader_.rfidAntennaPropLoader.rfidAngleSpan->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].angleSpan));
  
    loader_.rfidAntennaPropLoader.rfidOrientation->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].pose.theta));
  
    loader_.rfidAntennaPropLoader.rfidSignalCutoff->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].signalCutoff));
  
    loader_.rfidAntennaPropLoader.rfidFrequency->setText(
      QString().setNum(new_robot_msg_.rfidSensors[frameId].frequency));
    
    loader_.rfidAntennaPropLoader.setWindowTitle(
      QApplication::translate(
        "RfidAntennaProperties", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_rfid_ = item;
    
    loader_.rfidAntennaPropLoader.show();
  }
  
  /**
  @brief Returns the ID of a laser sensor
  @param frameId [QString] The frame id of the laser sensor 
  @return int
  **/
  int CRobotCreatorConnector::searchLaser(QString frameId)
  {
    for(unsigned int i = 0 ; i < new_robot_msg_.laserSensors.size() ; i++)
    {
      if(new_robot_msg_.laserSensors[i].frame_id == frameId.toStdString())
      {
        return i;
      }
    }
    return -1;
  }
  
  /**
  @brief Returns the ID of a sonar sensor
  @param frameId [QString] The frame id of the sonar sensor 
  @return int
  **/
  int CRobotCreatorConnector::searchSonar(QString frameId)
  {
    for(unsigned int i = 0 ; i < new_robot_msg_.sonarSensors.size() ; i++)
    {
      if(new_robot_msg_.sonarSensors[i].frame_id == frameId.toStdString())
      {
        return i;
      }
    }
    return -1;
  }
  
  /**
  @brief Returns the ID of an rfid antenna sensor
  @param frameId [QString] The frame id of the rfid antenna sensor 
  @return int
  **/
  int CRobotCreatorConnector::searchRfid(QString frameId)
  {
    for(unsigned int i = 0 ; i < new_robot_msg_.rfidSensors.size() ; i++)
    {
      if(new_robot_msg_.rfidSensors[i].frame_id == frameId.toStdString())
      {
        return i;
      }
    }
    return -1;
  }
  
  /**
  @brief Updates the robot's tree widget
  @return void
  **/
  void CRobotCreatorConnector::updateRobotTree(void)
  {
    for(unsigned int i = 0 ; i < loader_.robotNode.childCount() ; i++)
    {
      if(loader_.robotNode.child(i)->text(0) == QString("Orientation"))
      {
        loader_.robotNode.child(i)->
          setText(1,QString().setNum(new_robot_msg_.initialPose.theta));
      }
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.laserSensors.size() ; i++)
    {
      addLaser(new_robot_msg_.laserSensors[i]);
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.sonarSensors.size() ; i++)
    {
      addSonar(new_robot_msg_.sonarSensors[i]);
    }
  }
  
  /**
  @brief Updates a tree item with a specific laser sensor
  @param item [QTreeWidgetItem*] The tree item that will be updated
  @param l [stdr_msgs::LaserSensorMsg] The laser sensor message
  @return void
  **/
  void CRobotCreatorConnector::updateLaserTree(
    QTreeWidgetItem *item,
    stdr_msgs::LaserSensorMsg l)
  {
    for(unsigned int i = 0 ; i < item->childCount() ; i++)
    {
      if(item->child(i)->text(0) == QString("Angle span"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxAngle - l.minAngle));
      }
      else if(item->child(i)->text(0) == QString("Orientation"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.theta));
      }
      else if(item->child(i)->text(0) == QString("Max range"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxRange));
      }
      else if(item->child(i)->text(0) == QString("Number of rays"))
      {
        item->child(i)->setText(1,QString().setNum(l.numRays));
      }
      else if(item->child(i)->text(0) == QString("Min range"))
      {
        item->child(i)->setText(1,QString().setNum(l.minRange));
      }
      else if(item->child(i)->text(0) == QString("Noise mean"))
      {
        item->child(i)->setText(1,QString().setNum(l.noise.noiseMean));
      }
      else if(item->child(i)->text(0) == QString("Noise std"))
      {
        item->child(i)->setText(1,QString().setNum(l.noise.noiseStd));
      }
      else if(item->child(i)->text(0) == QString("Pose - x"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.x));
      }
      else if(item->child(i)->text(0) == QString("Pose - y"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.y));
      }
      else if(item->child(i)->text(0) == QString("Frequency"))
      {
        item->child(i)->setText(1,QString().setNum(l.frequency));
      }
    }
  }
  
  /**
  @brief Called when the update button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateLaser(void)
  {
    unsigned int laserFrameId = searchLaser(current_laser_->text(0));
    if(laserFrameId == -1) 
    {
      return;
    }
    float max_range = 1000.0;
    float min_range = 0.0;
    for(unsigned int i = 0 ; i < current_laser_->childCount() ; i++)
    {
      
      //!< Laser angle span
      if(current_laser_->child(i)->text(0) == QString("Angle span"))
      {
        current_laser_->child(i)->setText(
          1,loader_.laserPropLoader.laserAngleSpan->text());
        float angleSpan = loader_.laserPropLoader.laserAngleSpan->
          text().toFloat() / 2.0;
        if( angleSpan <= 0 )
        {
          showMessage(QString("Laser angle span invalid :") + 
            QString().setNum(angleSpan * 2.0));
          return;
        }
        new_robot_msg_.laserSensors[laserFrameId].minAngle = - angleSpan;
        new_robot_msg_.laserSensors[laserFrameId].maxAngle = angleSpan;
      }
      
      //!< Laser orientation
      else if(current_laser_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = loader_.laserPropLoader.laserOrientation->
          text().toFloat();
        current_laser_->child(i)->setText(
          1,QString().setNum(orientation));
        new_robot_msg_.laserSensors[laserFrameId].minAngle += orientation;
        new_robot_msg_.laserSensors[laserFrameId].maxAngle += orientation;
        new_robot_msg_.laserSensors[laserFrameId].pose.theta = 
          orientation;
      }
      
      //!< Laser max range
      else if(current_laser_->child(i)->text(0) == QString("Max range"))
      {
        max_range = loader_.laserPropLoader.laserMaxDistance->
          text().toFloat();
        if( max_range <= 0 )
        {
          showMessage(QString("Laser maximum range invalid :") + 
            QString().setNum(max_range));
          return;
        }
        if( max_range < min_range )
        {
          showMessage(QString("Laser maximum range lower than minimum range"));
          return;
        }
        current_laser_->child(i)->setText(1,QString().setNum(max_range));
        new_robot_msg_.laserSensors[laserFrameId].maxRange = max_range;
      }
      
      //!< Laser number of rays
      else if(current_laser_->child(i)->text(0) == QString("Number of rays"))
      {
        int rays = loader_.laserPropLoader.laserRays->text().toFloat();
        if( rays <= 0 )
        {
          showMessage(QString("Laser rays number invalid :") + 
            QString().setNum(rays));
          return;
        }
        current_laser_->child(i)->setText(1,QString().setNum(rays));
        new_robot_msg_.laserSensors[laserFrameId].numRays = rays;
      }
      
      //!< Laser minimum range
      else if(current_laser_->child(i)->text(0) == QString("Min range"))
      {
        min_range = loader_.laserPropLoader.laserMinDistance->
          text().toFloat();
        if( min_range < 0 )
        {
          showMessage(QString("Laser minimum range invalid :") + 
            QString().setNum(min_range));
          return;
        }
        if( min_range > max_range )
        {
          showMessage(QString("Laser minimum range higher than maximum range"));
          return;
        }
        current_laser_->child(i)->setText(1,QString().setNum(min_range));
        new_robot_msg_.laserSensors[laserFrameId].minRange = min_range;
      }
      
      //!< Laser mean noise
      else if(current_laser_->child(i)->text(0) == QString("Noise mean"))
      {
        float noiseMean = loader_.laserPropLoader.laserNoiseMean->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(noiseMean));
        new_robot_msg_.laserSensors[laserFrameId].noise.noiseMean = 
          noiseMean;
      }
      
      //!< Laser std noise
      else if(current_laser_->child(i)->text(0) == QString("Noise std"))
      {
        float noiseStd = 
          loader_.laserPropLoader.laserNoiseStd->text().toFloat();
        if( noiseStd < 0 )
        {
          showMessage(QString("Laser standard deviation of noise invalid :") + 
            QString().setNum(noiseStd));
          return;
        }
        current_laser_->child(i)->setText(1,QString().setNum(noiseStd));
        new_robot_msg_.laserSensors[laserFrameId].noise.noiseStd = 
          noiseStd;
      }
      
      //!< Laser pose - x coordinate
      else if(current_laser_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.laserPropLoader.laserTranslationX->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.laserSensors[laserFrameId].pose.x = dx;
      }
      
      //!< Laser pose - y coordinate
      else if(current_laser_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.laserPropLoader.laserTranslationY->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.laserSensors[laserFrameId].pose.y = dy;
      }
      
      //!< Laser publishing frequency
      else if(current_laser_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = loader_.laserPropLoader.laserFrequency->
          text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Laser publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_laser_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.laserSensors[laserFrameId].frequency = frequency;
      }
    }

    loader_.laserPropLoader.hide();
    
    updateRobotPreview();
  }
  
  /**
  @brief Called when the update button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateSonar(void)
  {
    unsigned int frameId=searchSonar(current_sonar_->text(0));
    if(frameId == -1)
    {
      return;
    }
    float min_range = 0;
    float max_range = 10000.0;
    for(unsigned int i = 0 ; i < current_sonar_->childCount() ; i++)
    {
      //!< Sonar cone span
      if(current_sonar_->child(i)->text(0) == QString("Cone span"))
      {
        float cone_span = 
          loader_.sonarPropLoader.sonarConeSpan->text().toFloat();
        if( cone_span <= 0 )
        {
          showMessage(QString("Sonar cone span invalid :") + 
            QString().setNum(cone_span));
          return;
        }
        current_sonar_->child(i)->setText(1,QString().setNum(cone_span));
        new_robot_msg_.sonarSensors[frameId].coneAngle = cone_span;
      }
      
      //!< Sonar orientation
      else if(current_sonar_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.sonarPropLoader.sonarOrientation->text().toFloat();
        current_sonar_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.sonarSensors[frameId].pose.theta = orientation;
      }
      
      //!< Sonar max range
      else if(current_sonar_->child(i)->text(0) == QString("Max range"))
      {
        max_range = loader_.sonarPropLoader.sonarMaxDistance->text().toFloat();
        if( max_range <= 0 )
        {
          showMessage(QString("Sonar maximum range invalid :") + 
            QString().setNum(max_range));
          return;
        }
        if( max_range < min_range )
        {
          showMessage(QString("Sonar maximum range lower than minimum range."));
          return;
        }
        current_sonar_->child(i)->
          setText(1,QString().setNum(max_range));
        new_robot_msg_.sonarSensors[frameId].maxRange = max_range;
      }
      
      //!< Sonar min range
      else if(current_sonar_->child(i)->text(0) == QString("Min range"))
      {
        min_range = loader_.sonarPropLoader.sonarMinDistance->text().toFloat();
        if( min_range < 0 )
        {
          showMessage(QString("Sonar minimum range invalid :") + 
            QString().setNum(min_range));
          return;
        }
        if( max_range < min_range )
        {
          showMessage(
            QString("Sonar minimum range higher than maximum range."));
          return;
        }
        current_sonar_->child(i)->setText(1,QString().setNum(min_range));
        new_robot_msg_.sonarSensors[frameId].minRange = min_range;
      }
      
      //!< Sonar noise mean
      else if(current_sonar_->child(i)->text(0) == QString("Noise mean"))
      {
        float noiseMean = 
          loader_.sonarPropLoader.sonarNoiseMean->text().toFloat();
        current_sonar_->child(i)->
          setText(1,QString().setNum(noiseMean));
        new_robot_msg_.sonarSensors[frameId].noise.noiseMean = noiseMean;
      }
      
      //!< Sonar noise standard deviation
      else if(current_sonar_->child(i)->text(0) == QString("Noise std"))
      {
        float noiseStd = 
          loader_.sonarPropLoader.sonarNoiseStd->text().toFloat();
        if( noiseStd < 0 )
        {
          showMessage(QString("Sonar noise standard deviation invalid :") + 
            QString().setNum(noiseStd));
          return;
        }
        current_sonar_->
          child(i)->setText(1,QString().setNum(noiseStd));
        new_robot_msg_.sonarSensors[frameId].noise.noiseStd = noiseStd;
      }
      
      //!< Sonar pose - x coordinate
      else if(current_sonar_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.sonarPropLoader.sonarX->text().toFloat();
        current_sonar_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.sonarSensors[frameId].pose.x = dx;
      }
      
      //!< Sonar pose - y coordinate
      else if(current_sonar_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.sonarPropLoader.sonarY->text().toFloat();
        current_sonar_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.sonarSensors[frameId].pose.y = dy;
      }
      
      //!< Sonar publishing frequency
      else if(current_sonar_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.sonarPropLoader.sonarFrequency->text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Sonar publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_sonar_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.sonarSensors[frameId].frequency = frequency;
      }
    }

    loader_.sonarPropLoader.hide();
    
    updateRobotPreview();
  }
  
  /**
  @brief Updates a tree item with a specific sonar sensor
  @param item [QTreeWidgetItem*] The tree item that will be updated
  @param l [stdr_msgs::SonarSensorMsg] The sonar sensor message
  @return void
  **/
  void CRobotCreatorConnector::updateSonarTree(
    QTreeWidgetItem *item,
    stdr_msgs::SonarSensorMsg l)
  {
    unsigned int frameId=searchSonar(item->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < item->childCount() ; i++)
    {
      if(item->child(i)->text(0) == QString("Cone span"))
      {
        item->child(i)->setText(1,QString().setNum(l.coneAngle));
      }
      else if(item->child(i)->text(0) == QString("Orientation"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.theta));
      }
      else if(item->child(i)->text(0) == QString("Max range"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxRange));
      }
      else if(item->child(i)->text(0) == QString("Min range"))
      {
        item->child(i)->setText(1,QString().setNum(l.minRange));
      }
      else if(item->child(i)->text(0) == QString("Noise mean"))
      {
        item->child(i)->setText(1,QString().setNum(l.noise.noiseMean));
      }
      else if(item->child(i)->text(0) == QString("Noise std"))
      {
        item->child(i)->setText(1,QString().setNum(l.noise.noiseStd));
      }
      else if(item->child(i)->text(0) == QString("Pose - x"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.x));
      }
      else if(item->child(i)->text(0) == QString("Pose - y"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.y));
      }
      else if(item->child(i)->text(0) == QString("Frequency"))
      {
        item->child(i)->setText(1,QString().setNum(l.frequency));
      }
    }
  }
  
  /**
  @brief Called when the update button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateRfid(void)
  {
    unsigned int frameId = searchRfid(current_rfid_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_rfid_->childCount() ; i++)
    {
      
      //!< Rfid antenna angle span
      if(current_rfid_->child(i)->text(0) == QString("Angle span"))
      {
        float angle_span = 
          loader_.rfidAntennaPropLoader.rfidAngleSpan->text().toFloat();
        if( angle_span <= 0 )
        {
          showMessage(QString("Rfid antenna angle span invalid :") + 
            QString().setNum(angle_span));
          return;
        }
        current_rfid_->child(i)->setText(1,QString().setNum(angle_span));
        new_robot_msg_.rfidSensors[frameId].angleSpan = angle_span;
      }
      
      //!< Rfid antenna orientation
      else if(current_rfid_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.rfidAntennaPropLoader.rfidOrientation->
            text().toFloat();
        current_rfid_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.rfidSensors[frameId].pose.theta = orientation;
      }
      
      //!< Rfid antenna max range
      else if(current_rfid_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.rfidAntennaPropLoader.rfidMaxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("Rfid antenna maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_rfid_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.rfidSensors[frameId].maxRange = maxRange;
      }
      
      //!< Rfid antenna pose - x coordinate
      else if(current_rfid_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.rfidAntennaPropLoader.rfidX->text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.rfidSensors[frameId].pose.x = dx;
      }
      
      //!< Rfid antenna pose - y coordinate
      else if(current_rfid_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.rfidAntennaPropLoader.rfidY->text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.rfidSensors[frameId].pose.y = dy;
      }
      
      //!< Rfid antenna signal cutoff
      else if(current_rfid_->child(i)->text(0) == QString("Signal cutoff"))
      {
        float signal = 
          loader_.rfidAntennaPropLoader.rfidSignalCutoff->
            text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(signal));
        new_robot_msg_.rfidSensors[frameId].signalCutoff = signal;
      }
      
      //!< Rfid antenna publishing frequency
      else if(current_rfid_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.rfidAntennaPropLoader.rfidFrequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Rfid antenna publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_rfid_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.rfidSensors[frameId].frequency = frequency;
      }
    }

    loader_.rfidAntennaPropLoader.hide();
    
    updateRobotPreview();
  }
  
  /**
  @brief Shows the edit robot widget
  @return void
  **/
  void CRobotCreatorConnector::editRobot(void)
  {
    loader_.robotPropLoader.robotOrientation->
      setText(QString().setNum(new_robot_msg_.initialPose.theta));
    loader_.robotPropLoader.show();
  }
  
  /**
  @brief Called when the update button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateRobot(void)
  {
    for(unsigned int i = 0 ; i < loader_.robotNode.childCount() ; i++)
    {
      if(loader_.robotNode.child(i)->text(0) == QString("Orientation"))
      {
        loader_.robotNode.child(i)->
          setText(1,loader_.robotPropLoader.robotOrientation->text());
        new_robot_msg_.initialPose.theta = 
          loader_.robotPropLoader.robotOrientation->text().toFloat();
      }
    }

    loader_.robotPropLoader.hide();
    
    updateRobotPreview();
  }

  /**
  @brief Deletes a specific tree item and it's children recursively
  @param item [QTreeWidgetItem*] The item to be erased
  @return void
  **/
  void CRobotCreatorConnector::deleteTreeNode(QTreeWidgetItem *item)
  {
    int count = item->childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      deleteTreeNode(item->child(i));
    }
    delete item;
  }
    
  /**
  @brief Updates the robot's preview
  @return void
  **/
  void CRobotCreatorConnector::updateRobotPreview(void)
  {
    
    loader_.robotPreviewImage.fill(QColor(220,220,220,1));
    
    climax_ = -1;
    if(climax_ < new_robot_msg_.footprint.radius) 
    {
      climax_ = new_robot_msg_.footprint.radius;
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.laserSensors.size() ; i++)
    {
      if(climax_ < (new_robot_msg_.laserSensors[i].maxRange + 
          new_robot_msg_.laserSensors[i].pose.x))
      { 
        climax_ = new_robot_msg_.laserSensors[i].maxRange + 
          new_robot_msg_.laserSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.laserSensors[i].maxRange + 
          new_robot_msg_.laserSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.laserSensors[i].maxRange + 
          new_robot_msg_.laserSensors[i].pose.y;  
      }
      if(climax_ < (new_robot_msg_.laserSensors[i].maxRange - 
          new_robot_msg_.laserSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.laserSensors[i].maxRange - 
          new_robot_msg_.laserSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.laserSensors[i].maxRange - 
          new_robot_msg_.laserSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.laserSensors[i].maxRange - 
          new_robot_msg_.laserSensors[i].pose.y;  
      }
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.sonarSensors.size() ; i++)
    {
      if(climax_ < (new_robot_msg_.sonarSensors[i].maxRange + 
          new_robot_msg_.sonarSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.sonarSensors[i].maxRange + 
          new_robot_msg_.sonarSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.sonarSensors[i].maxRange + 
          new_robot_msg_.sonarSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.sonarSensors[i].maxRange + 
          new_robot_msg_.sonarSensors[i].pose.y;  
      }
      if(climax_ < (new_robot_msg_.sonarSensors[i].maxRange - 
          new_robot_msg_.sonarSensors[i].pose.x) )
      {  
        climax_ = new_robot_msg_.sonarSensors[i].maxRange - 
          new_robot_msg_.sonarSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.sonarSensors[i].maxRange - 
          new_robot_msg_.sonarSensors[i].pose.y) )
      {  
        climax_ = new_robot_msg_.sonarSensors[i].maxRange - 
          new_robot_msg_.sonarSensors[i].pose.y;  
      }
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.rfidSensors.size() ; i++)
    {
      if(climax_ < (new_robot_msg_.rfidSensors[i].maxRange + 
          new_robot_msg_.rfidSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.rfidSensors[i].maxRange + 
          new_robot_msg_.rfidSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.rfidSensors[i].maxRange + 
          new_robot_msg_.rfidSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.rfidSensors[i].maxRange + 
          new_robot_msg_.rfidSensors[i].pose.y;  
      }
      if(climax_ < (new_robot_msg_.rfidSensors[i].maxRange - 
          new_robot_msg_.rfidSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.rfidSensors[i].maxRange - 
          new_robot_msg_.rfidSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.rfidSensors[i].maxRange - 
          new_robot_msg_.rfidSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.rfidSensors[i].maxRange - 
          new_robot_msg_.rfidSensors[i].pose.y;  
      }
    }
    
    climax_ = 230.0 / climax_;
    drawRobot(new_robot_msg_.footprint.radius);
    drawLasers();
    drawSonars();
    drawRfidAntennas();
    
    loader_.robotTreeWidget->resizeColumnToContents(0);
    loader_.robotTreeWidget->resizeColumnToContents(1);
    loader_.robotTreeWidget->resizeColumnToContents(2);
    loader_.robotTreeWidget->resizeColumnToContents(3);
    loader_.robotTreeWidget->resizeColumnToContents(4);
  }
  
  /**
  @brief Draws a circular robot
  @param radius [float] The robot radius 
  @return void
  **/
  void CRobotCreatorConnector::drawRobot(float radius)
  {
    QPainter painter(&loader_.robotPreviewImage);
    painter.setPen(Qt::blue);
    painter.drawEllipse(
              250 - radius * climax_,
              250 - radius * climax_,
              radius * climax_ * 2,
              radius * climax_ * 2);
    painter.drawLine(  
              250,
              250,
              250 + radius * climax_ * 1.05 * cos(
                new_robot_msg_.initialPose.theta / 180.0 * STDR_PI),
              250 - radius * climax_ * 1.05 * sin(
                new_robot_msg_.initialPose.theta / 180.0 * STDR_PI));
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
  }
  
  /**
  @brief Draws a robot with a specific footprint
  @param geometry [std::vector<std::pair<float,float> >] The robot footprint 
  @return void
  **/
  void CRobotCreatorConnector::drawRobot(
    std::vector<std::pair<float,float> > geometry)
  {
    
  }

  /**
  @brief Draws the robot's lasers
  @return void
  **/
  void CRobotCreatorConnector::drawLasers(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    QBrush brush(QColor(0,200,0,50));
    painter.setBrush(brush);
    float robotOrientation = 
      new_robot_msg_.initialPose.theta / 180.0 * STDR_PI;
    
    for(unsigned int i = 0 ; i < new_robot_msg_.laserSensors.size() ; i++)
    {
      
      float laserx = new_robot_msg_.laserSensors[i].pose.x;
      float lasery = new_robot_msg_.laserSensors[i].pose.y;
      float newx = laserx * cos(robotOrientation) - 
        lasery * sin(robotOrientation);
      float newy = laserx * sin(robotOrientation) + 
        lasery * cos(robotOrientation);
      
      painter.drawPie(
        250 - new_robot_msg_.laserSensors[i].maxRange * climax_ +   
          newx * climax_,
        250 - new_robot_msg_.laserSensors[i].maxRange * climax_ - 
          newy * climax_,
        new_robot_msg_.laserSensors[i].maxRange * climax_ * 2,
        new_robot_msg_.laserSensors[i].maxRange * climax_ * 2,
        (new_robot_msg_.laserSensors[i].minAngle + 
          new_robot_msg_.initialPose.theta) * 16,
        new_robot_msg_.laserSensors[i].maxAngle * 16 - 
          new_robot_msg_.laserSensors[i].minAngle * 16);
    }
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
  }
  
  /**
  @brief Draws the robot's sonars
  @return void
  **/
  void CRobotCreatorConnector::drawSonars(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    QBrush brush(QColor(200,0,0,50));
    painter.setBrush(brush);
    float robotOrientation =  
      new_robot_msg_.initialPose.theta / 180.0 * STDR_PI;
    
    for(unsigned int i = 0 ; i < new_robot_msg_.sonarSensors.size() ; i++)
    {
      
      float sonarx = new_robot_msg_.sonarSensors[i].pose.x;
      float sonary = new_robot_msg_.sonarSensors[i].pose.y;
      float newx = sonarx * cos(robotOrientation) - 
            sonary * sin(robotOrientation);
      float newy = sonarx * sin(robotOrientation) + 
            sonary * cos(robotOrientation);
      
      painter.drawPie(  
        250 - new_robot_msg_.sonarSensors[i].maxRange * climax_ + 
          newx * climax_,
        250 - new_robot_msg_.sonarSensors[i].maxRange * climax_ - 
          newy * climax_,
        new_robot_msg_.sonarSensors[i].maxRange * climax_ * 2,
        new_robot_msg_.sonarSensors[i].maxRange * climax_ * 2,
        (new_robot_msg_.sonarSensors[i].pose.theta - 
          new_robot_msg_.sonarSensors[i].coneAngle / 2.0 + 
          new_robot_msg_.initialPose.theta) * 16,
        (new_robot_msg_.sonarSensors[i].coneAngle) * 16);
    }
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
  }
  
  /**
  @brief Draws the robot's rfid antennas
  @return void
  **/
  void CRobotCreatorConnector::drawRfidAntennas(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    QBrush brush(QColor(0,0,200,20));
    painter.setBrush(brush);
    float robotOrientation = 
      new_robot_msg_.initialPose.theta / 180.0 * STDR_PI;
    
    for(unsigned int i = 0 ; i < new_robot_msg_.rfidSensors.size() ; i++)
    {
      
      float rfidx = new_robot_msg_.rfidSensors[i].pose.x;
      float rfidy = new_robot_msg_.rfidSensors[i].pose.y;
      float newx = 
        rfidx * cos(robotOrientation) - rfidy * sin(robotOrientation);
      float newy = 
        rfidx * sin(robotOrientation) + rfidy * cos(robotOrientation);
      
      painter.drawPie(  
        250 - new_robot_msg_.rfidSensors[i].maxRange * climax_ + newx * climax_,
        250 - new_robot_msg_.rfidSensors[i].maxRange * climax_ - newy * climax_,
        new_robot_msg_.rfidSensors[i].maxRange * climax_ * 2,
        new_robot_msg_.rfidSensors[i].maxRange * climax_ * 2,
        (new_robot_msg_.rfidSensors[i].pose.theta - 
          new_robot_msg_.rfidSensors[i].angleSpan / 2.0+
          new_robot_msg_.initialPose.theta) * 16,
        (new_robot_msg_.rfidSensors[i].angleSpan) * 16);
    }
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
  }

  /**
  @brief Called when the save robot button is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::saveRobot(void)
  {
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save File"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Yaml files (*.yaml)"));
    
    Q_EMIT saveRobotPressed(
      stdr_gui_tools::fixRobotAnglesToRad(new_robot_msg_),file_name);
  }
  
  /**
  @brief Called when the load robot button 
  @return void
  **/ 
  void CRobotCreatorConnector::getRobotFromYaml(void)
  {
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load robot"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Yaml Files (*.yaml)"));
    
    //!< Not a valid filename
    if (file_name.isEmpty()) { 
      return;
    }
    
    try {
      new_robot_msg_ = 
      stdr_robot::parser::yamlToRobotMsg(file_name.toStdString());
      
      new_robot_msg_ = stdr_gui_tools::fixRobotAnglesToDegrees(new_robot_msg_);

      CRobotCreatorConnector::laser_number = 0;
      CRobotCreatorConnector::sonar_number = 0;
      updateRobotTree();
      
    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
    updateRobotPreview(); 
  }
  
  /**
  @brief Called when the load robot in map button is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::loadRobot(void)
  {
    Q_EMIT loadRobotPressed(new_robot_msg_);
    loader_.hide();
  }
  
  /**
  @brief Sets the robot's initial pose
  @param x [float] x coordinate
  @param y [float] y coordinate
  @return void
  **/
  void CRobotCreatorConnector::setInitialPose(float x, float y)
  {
    new_robot_msg_.initialPose.x = x;
    new_robot_msg_.initialPose.y = y;
  }
  
  /**
  @brief Returns the created robot
  @return stdr_msgs::RobotMsg : The new robot
  **/
  stdr_msgs::RobotMsg CRobotCreatorConnector::getNewRobot(void)
  {
    return new_robot_msg_;
  }
  
  /**
  @brief Sets the created robot
  @param msg [stdr_msgs::RobotMsg] The new robot
  @return void
  **/
  void CRobotCreatorConnector::setNewRobot(stdr_msgs::RobotMsg msg)
  {
    new_robot_msg_=msg;
  }
  
  /**
  @brief Pops up a message box with a specific message
  @param msg [QString] Message to be shown
  @return void
  **/
  void CRobotCreatorConnector::showMessage(QString msg)
  {
    QMessageBox msgBox;
    msgBox.setText(msg);
    msgBox.exec();
  }
}
