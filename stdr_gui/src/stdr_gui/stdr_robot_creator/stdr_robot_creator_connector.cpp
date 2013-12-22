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
      loader_.saveRobotButton,SIGNAL(clicked(bool)),
      this,SLOT(saveRobot()));
    
    QObject::connect(
      loader_.loadRobotButton,SIGNAL(clicked(bool)),
      this,SLOT(loadRobot()));
    
    QObject::connect(
      loader_.addButton,SIGNAL(clicked(bool)),
      this,SLOT(getRobotFromYaml()));

    climax_ = - 1;
  }
  
  CRobotCreatorConnector::~CRobotCreatorConnector(void)
  {
    
  }
  
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

  void CRobotCreatorConnector::treeItemClicked( 
    QTreeWidgetItem * item, 
    int column)
  {
    //!< Robot edit clicked
    if(item == &loader_.robotNode && column == 2)
    {    
      editRobot();
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
      addRfidAntenna();
    }  
    //!< Erase a sonar
    if(item->parent() == &loader_.sonarsNode && column == 2)
    {
      eraseSonar(item);
    }  
    //!< Erase a rfid antenna
    if(item->parent() == &loader_.rfidAntennasNode && column == 2)
    {
      eraseRfid(item);
    }  
    //!< Edit a sonar
    if(item->parent() == &loader_.sonarsNode && column == 1)
    {
      editSonar(item);
    }  
    //!< Edit a rfid antenna  
    if(item->parent() == &loader_.rfidAntennasNode && column == 1)
    {
      editRfid(item);
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
  }
  
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
   
    try {
      //~ stdr_robot::parser::laserMsgToYaml(file_name_str,newRobotMsg);
    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }
  
  void CRobotCreatorConnector::loadLaser(QTreeWidgetItem *item)
  {
    //~ unsigned int laserFrameId = searchLaser(item->text(0));
    //~ if(laserFrameId == -1) 
    //~ {
      //~ return;
    //~ }  
    //~ QString file_name = QFileDialog::getSaveFileName(&loader_, 
      //~ tr("Save laser sensor"),
        //~ QString().fromStdString(
        //~ stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        //~ QString("/resources/"),
        //~ tr("Yaml files (*.yaml)"));
    //~ 
    //~ std::string file_name_str=file_name.toStdString();
   //~ 
    //~ try {
      //~ stdr_robot::parser::laserMsgToYaml(file_name_str,newRobotMsg);
    //~ }
    //~ catch(YAML::RepresentationException& e) {
      //~ ROS_ERROR("%s", e.what());
      //~ return;
    //~ }
    //~ updateRobotPreview(); 
  }
  
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
      
  void CRobotCreatorConnector::updateLaser(void)
  {
    unsigned int laserFrameId = searchLaser(current_laser_->text(0));
    if(laserFrameId == -1) 
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_laser_->childCount() ; i++)
    {
      if(current_laser_->child(i)->text(0) == QString("Angle span"))
      {
        current_laser_->child(i)->setText(
          1,loader_.laserPropLoader.laserAngleSpan->text());
        float angleSpan = loader_.laserPropLoader.laserAngleSpan->
          text().toFloat() / 2.0;
        new_robot_msg_.laserSensors[laserFrameId].minAngle = - angleSpan;
        new_robot_msg_.laserSensors[laserFrameId].maxAngle = angleSpan;
      }
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
      else if(current_laser_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = loader_.laserPropLoader.laserMaxDistance->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.laserSensors[laserFrameId].maxRange = maxRange;
      }
      else if(current_laser_->child(i)->text(0) == QString("Number of rays"))
      {
        int rays = loader_.laserPropLoader.laserRays->text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(rays));
        new_robot_msg_.laserSensors[laserFrameId].numRays = rays;
      }
      else if(current_laser_->child(i)->text(0) == QString("Min range"))
      {
        float minRange = loader_.laserPropLoader.laserMinDistance->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(minRange));
        new_robot_msg_.laserSensors[laserFrameId].minRange = minRange;
      }
      else if(current_laser_->child(i)->text(0) == QString("Noise mean"))
      {
        float noiseMean = loader_.laserPropLoader.laserNoiseMean->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(noiseMean));
        new_robot_msg_.laserSensors[laserFrameId].noise.noiseMean = 
          noiseMean;
      }
      else if(current_laser_->child(i)->text(0) == QString("Noise std"))
      {
        float noiseStd = 
          loader_.laserPropLoader.laserNoiseStd->text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(noiseStd));
        new_robot_msg_.laserSensors[laserFrameId].noise.noiseStd = 
          noiseStd;
      }
      else if(current_laser_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.laserPropLoader.laserTranslationX->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.laserSensors[laserFrameId].pose.x = dx;
      }
      else if(current_laser_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.laserPropLoader.laserTranslationY->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.laserSensors[laserFrameId].pose.y = dy;
      }
      else if(current_laser_->child(i)->text(0) == QString("Frequency"))
      {
        float dy = loader_.laserPropLoader.laserFrequency->
          text().toFloat();
        current_laser_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.laserSensors[laserFrameId].frequency = dy;
      }
    }

    loader_.laserPropLoader.hide();
    
    updateRobotPreview();
  }

  void CRobotCreatorConnector::updateSonar(void)
  {
    unsigned int frameId=searchSonar(current_sonar_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_sonar_->childCount() ; i++)
    {
      if(current_sonar_->child(i)->text(0) == QString("Cone span"))
      {
        current_sonar_->child(i)->
          setText(1,loader_.sonarPropLoader.sonarConeSpan->text());
        new_robot_msg_.sonarSensors[frameId].coneAngle = 
          loader_.sonarPropLoader.sonarConeSpan->text().toFloat();
      }
      else if(current_sonar_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.sonarPropLoader.sonarOrientation->text().toFloat();
        current_sonar_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.sonarSensors[frameId].pose.theta = orientation;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.sonarPropLoader.sonarMaxDistance->text().toFloat();
        current_sonar_->child(i)->
          setText(1,QString().setNum(maxRange));
        new_robot_msg_.sonarSensors[frameId].maxRange = maxRange;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Min range"))
      {
        float minRange = 
          loader_.sonarPropLoader.sonarMinDistance->text().toFloat();
        current_sonar_->child(i)->setText(1,QString().setNum(minRange));
        new_robot_msg_.sonarSensors[frameId].minRange = minRange;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Noise mean"))
      {
        float noiseMean = 
          loader_.sonarPropLoader.sonarNoiseMean->text().toFloat();
        current_sonar_->child(i)->
          setText(1,QString().setNum(noiseMean));
        new_robot_msg_.sonarSensors[frameId].noise.noiseMean = noiseMean;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Noise std"))
      {
        float noiseStd = 
          loader_.sonarPropLoader.sonarNoiseStd->text().toFloat();
        current_sonar_->
          child(i)->setText(1,QString().setNum(noiseStd));
        new_robot_msg_.sonarSensors[frameId].noise.noiseStd = noiseStd;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.sonarPropLoader.sonarX->text().toFloat();
        current_sonar_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.sonarSensors[frameId].pose.x = dx;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.sonarPropLoader.sonarY->text().toFloat();
        current_sonar_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.sonarSensors[frameId].pose.y = dy;
      }
      else if(current_sonar_->child(i)->text(0) == QString("Frequency"))
      {
        float dy = 
          loader_.sonarPropLoader.sonarFrequency->text().toFloat();
        current_sonar_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.sonarSensors[frameId].frequency = dy;
      }
    }

    loader_.sonarPropLoader.hide();
    
    updateRobotPreview();
  }
  
  void CRobotCreatorConnector::updateRfid(void)
  {
    unsigned int frameId = searchRfid(current_rfid_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_rfid_->childCount() ; i++)
    {
      if(current_rfid_->child(i)->text(0) == QString("Angle span"))
      {
        current_rfid_->child(i)->setText(1,
          loader_.rfidAntennaPropLoader.rfidAngleSpan->text());
        new_robot_msg_.rfidSensors[frameId].angleSpan = 
          loader_.rfidAntennaPropLoader.rfidAngleSpan->
            text().toFloat();
      }
      else if(current_rfid_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.rfidAntennaPropLoader.rfidOrientation->
            text().toFloat();
        current_rfid_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.rfidSensors[frameId].pose.theta = orientation;
      }
      else if(current_rfid_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.rfidAntennaPropLoader.rfidMaxDistance->
            text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.rfidSensors[frameId].maxRange = maxRange;
      }
      else if(current_rfid_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.rfidAntennaPropLoader.rfidX->text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.rfidSensors[frameId].pose.x = dx;
      }
      else if(current_rfid_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.rfidAntennaPropLoader.rfidY->text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.rfidSensors[frameId].pose.y = dy;
      }
      else if(current_rfid_->child(i)->text(0) == QString("Signal cutoff"))
      {
        float signal = 
          loader_.rfidAntennaPropLoader.rfidSignalCutoff->
            text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(signal));
        new_robot_msg_.rfidSensors[frameId].signalCutoff = signal;
      }
      else if(current_rfid_->child(i)->text(0) == QString("Frequency"))
      {
        float signal = 
          loader_.rfidAntennaPropLoader.rfidFrequency->
            text().toFloat();
        current_rfid_->child(i)->setText(1,QString().setNum(signal));
        new_robot_msg_.rfidSensors[frameId].frequency = signal;
      }
    }

    loader_.rfidAntennaPropLoader.hide();
    
    updateRobotPreview();
  }
  
  void CRobotCreatorConnector::editRobot(void)
  {
    loader_.robotPropLoader.robotOrientation->
      setText(QString().setNum(new_robot_msg_.initialPose.theta));
    loader_.robotPropLoader.show();
  }
  
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

  void CRobotCreatorConnector::deleteTreeNode(QTreeWidgetItem *item)
  {
    int count = item->childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      deleteTreeNode(item->child(i));
    }
    delete item;
  }
    
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
  }
  
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
  
  void CRobotCreatorConnector::drawRobot(float length,float width)
  {
    
  }
  
  void CRobotCreatorConnector::drawRobot(
    std::vector<std::pair<float,float> > geometry)
  {
    
  }

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
  
  void CRobotCreatorConnector::getRobotFromYaml(void)
  {
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load robot"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Yaml Files (*.yaml)"));
    
    if (file_name.isEmpty()) { // Not a valid filename
      return;
    }
    
    try {
      new_robot_msg_ = 
      stdr_robot::parser::yamlToRobotMsg(file_name.toStdString()); // need to fix angles from rads to deg

      new_robot_msg_ = stdr_gui_tools::fixRobotAnglesToDegrees(new_robot_msg_);

    }
    catch(YAML::RepresentationException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
    updateRobotPreview(); 
  }
  
  void CRobotCreatorConnector::loadRobot(void)
  {
    Q_EMIT loadRobotPressed(new_robot_msg_);
    loader_.hide();
  }
  
  void CRobotCreatorConnector::setInitialPose(float x, float y)
  {
    new_robot_msg_.initialPose.x = x;
    new_robot_msg_.initialPose.y = y;
  }
  
  stdr_msgs::RobotMsg CRobotCreatorConnector::getNewRobot(void)
  {
    return new_robot_msg_;
  }
}
