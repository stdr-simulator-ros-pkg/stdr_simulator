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
  
  int CRobotCreatorConnector::laser_number = -1;
  int CRobotCreatorConnector::sonar_number = -1;
  int CRobotCreatorConnector::rfid_number = -1;
  int CRobotCreatorConnector::co2_sensors_number = -1;
  int CRobotCreatorConnector::thermal_sensors_number = -1;
  int CRobotCreatorConnector::sound_sensors_number = -1;
  
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
      loader_.laserPropLoader.refresh_laser,SIGNAL(clicked(bool)),
      this,SLOT(updateLaserOpen()));
    
    QObject::connect(
      loader_.robotPropLoader.updateButton,SIGNAL(clicked(bool)),
      this,SLOT(updateRobot()));
    QObject::connect(
      loader_.robotPropLoader.refresh_robot,SIGNAL(clicked(bool)),
      this,SLOT(updateRobotOpen()));
    
    QObject::connect(
      loader_.sonarPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateSonar()));
    QObject::connect(
      loader_.sonarPropLoader.refresh_sonar,SIGNAL(clicked(bool)),
      this,SLOT(updateSonarOpen()));
      
    QObject::connect(
      loader_.rfidAntennaPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateRfidAntenna()));
    QObject::connect(
      loader_.rfidAntennaPropLoader.refreshRfid,SIGNAL(clicked(bool)),
      this,SLOT(updateRfidAntennaOpen()));
      
    QObject::connect(
      loader_.co2SensorPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateCO2Sensor()));
    QObject::connect(
      loader_.co2SensorPropLoader.refreshButton,SIGNAL(clicked(bool)),
      this,SLOT(updateCO2SensorOpen()));

    QObject::connect(
      loader_.thermalSensorPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateThermalSensor()));
    QObject::connect(
      loader_.thermalSensorPropLoader.refreshButton,SIGNAL(clicked(bool)),
      this,SLOT(updateThermalSensorOpen()));
    
    QObject::connect(
      loader_.soundSensorPropLoader.pushButton,SIGNAL(clicked(bool)),
      this,SLOT(updateSoundSensor()));
    QObject::connect(
      loader_.soundSensorPropLoader.refreshButton,SIGNAL(clicked(bool)),
      this,SLOT(updateSoundSensorOpen()));
      
    QObject::connect(
      loader_.robotFootLoader.updateButton,SIGNAL(clicked(bool)),
      this,SLOT(updateFootprintPoint()));
    QObject::connect(
      loader_.robotFootLoader.refresh_robot,SIGNAL(clicked(bool)),
      this,SLOT(updateFootprintPointOpen()));

    QObject::connect(
      loader_.kinematicPropLoader.updateButton,SIGNAL(clicked(bool)),
      this,SLOT(updateKinematicModel()));
    
    QObject::connect(
      loader_.loadRobotButton,SIGNAL(clicked(bool)),
      this,SLOT(loadRobot()));
    

    climax_ = - 1;
    sonar_hightlight_id_ = -1;
    laser_hightlight_id_ = -1;
    rfid_antenna_hightlight_id_ = -1;
    co2_sensor_hightlight_id_ = -1;
    thermal_sensor_hightlight_id_ = -1;
    sound_sensor_hightlight_id_ = -1;
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

    loader_.robotInfoRadius.setText(0,"Radius");
    loader_.robotInfoRadius.setText(1,"0.15");
    loader_.robotInfoOrientation.setText(0,"Orientation");
    loader_.robotInfoOrientation.setText(1,"0");
    
    new_robot_msg_.footprint.radius = 0.15;
    
    unsigned int laserCount = loader_.lasersNode.childCount();
    unsigned int sonarCount = loader_.sonarsNode.childCount();
    unsigned int rfidCount = loader_.rfidAntennasNode.childCount();
    unsigned int co2SensorsCount = loader_.co2SensorsNode.childCount();
    unsigned int thermalSensorsCount = loader_.thermalSensorsNode.childCount();
    unsigned int soundSensorsCount = loader_.soundSensorsNode.childCount();
    unsigned int footprintCount = loader_.robotInfoFootprint.childCount();
    
    for(int i = laserCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.lasersNode.child(i));
    for(int i = sonarCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.sonarsNode.child(i));
    for(int i = rfidCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.rfidAntennasNode.child(i));
    for(int i = co2SensorsCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.co2SensorsNode.child(i));
    for(int i = thermalSensorsCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.thermalSensorsNode.child(i));
    for(int i = soundSensorsCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.soundSensorsNode.child(i));
    for(int i = footprintCount - 1 ; i >= 0 ; i--)
      deleteTreeNode(loader_.robotInfoFootprint.child(i));
    
    CRobotCreatorConnector::laser_number = 0;
    CRobotCreatorConnector::sonar_number = 0;
    CRobotCreatorConnector::rfid_number = 0;
    CRobotCreatorConnector::co2_sensors_number = 0;
    CRobotCreatorConnector::thermal_sensors_number = 0;
    CRobotCreatorConnector::sound_sensors_number = 0;
    
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
    
    //!< Laser clicked
    if(item->parent() == &loader_.lasersNode)
    {
      unsigned int laserFrameId = searchLaser(item->text(0));
      if(laserFrameId == -1) 
      {
        ROS_ERROR("Something went terribly wrong...");
      }
      laser_hightlight_id_ = laserFrameId;
      sonar_hightlight_id_ = -1;
      rfid_antenna_hightlight_id_ = -1;
      co2_sensor_hightlight_id_ = -1;
      thermal_sensor_hightlight_id_ = -1;
      sound_sensor_hightlight_id_ = -1;
    }  
    //!< Sonar clicked
    if(item->parent() == &loader_.sonarsNode)
    {
      unsigned int sonarFrameId = searchSonar(item->text(0));
      if(sonarFrameId == -1) 
      {
        ROS_ERROR("Something went terribly wrong...");
      }
      sonar_hightlight_id_ = sonarFrameId;
      laser_hightlight_id_ = -1;
      rfid_antenna_hightlight_id_ = -1;
      co2_sensor_hightlight_id_ = -1;
      thermal_sensor_hightlight_id_ = -1;
      sound_sensor_hightlight_id_ = -1;
    }  
    //!< Rfid antenna clicked
    if(item->parent() == &loader_.rfidAntennasNode)
    {
      unsigned int frameId = searchRfid(item->text(0));
      if(frameId == -1) 
      {
        ROS_ERROR("Something went terribly wrong...");
      }
      rfid_antenna_hightlight_id_ = frameId;
      laser_hightlight_id_ = -1;
      sonar_hightlight_id_ = -1;
      co2_sensor_hightlight_id_ = -1;
      thermal_sensor_hightlight_id_ = -1;
      sound_sensor_hightlight_id_ = -1;
    }  
    //!< CO2 sensor clicked
    if(item->parent() == &loader_.co2SensorsNode)
    {
      unsigned int frameId = searchCO2Sensor(item->text(0));
      if(frameId == -1) 
      {
        ROS_ERROR("Something went terribly wrong...");
      }
      rfid_antenna_hightlight_id_ = -1;
      laser_hightlight_id_ = -1;
      sonar_hightlight_id_ = -1;
      co2_sensor_hightlight_id_ = frameId;
      thermal_sensor_hightlight_id_ = -1;
      sound_sensor_hightlight_id_ = -1;
    }  
    //!< Thermal sensor clicked
    if(item->parent() == &loader_.thermalSensorsNode)
    {
      unsigned int frameId = searchThermalSensor(item->text(0));
      if(frameId == -1) 
      {
        ROS_ERROR("Something went terribly wrong...");
      }
      rfid_antenna_hightlight_id_ = -1;
      laser_hightlight_id_ = -1;
      sonar_hightlight_id_ = -1;
      co2_sensor_hightlight_id_ = -1;
      thermal_sensor_hightlight_id_ = frameId;
      sound_sensor_hightlight_id_ = -1;
    }  
    //!< Sound sensor clicked
    if(item->parent() == &loader_.soundSensorsNode)
    {
      unsigned int frameId = searchSoundSensor(item->text(0));
      if(frameId == -1) 
      {
        ROS_ERROR("Something went terribly wrong...");
      }
      rfid_antenna_hightlight_id_ = -1;
      laser_hightlight_id_ = -1;
      sonar_hightlight_id_ = -1;
      co2_sensor_hightlight_id_ = -1;
      thermal_sensor_hightlight_id_ = -1;
      sound_sensor_hightlight_id_ = frameId;
    }  
    
    updateRobotPreview();
    
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
      addRfidAntenna();
    }  
    //!< Add a CO2 sensor
    if(item == &loader_.co2SensorsNode && column == 2)
    {
      addCO2Sensor();
    }  
    //!< Add a thermal sensor
    if(item == &loader_.thermalSensorsNode && column == 2)
    {
      addThermalSensor();
    }  
    //!< Add a sound sensor
    if(item == &loader_.soundSensorsNode && column == 2)
    {
      addSoundSensor();
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
    //!< Erase a co2 sensor
    if(item->parent() == &loader_.co2SensorsNode && column == 2)
    {
      eraseCO2Sensor(item);
    }  
    //!< Erase a thermal sensor
    if(item->parent() == &loader_.thermalSensorsNode && column == 2)
    {
      eraseThermalSensor(item);
    }  
    //!< Erase a sound sensor
    if(item->parent() == &loader_.soundSensorsNode && column == 2)
    {
      eraseSoundSensor(item);
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
    //!< Edit a co2 sensor
    if(item->parent() == &loader_.co2SensorsNode && column == 1)
    {
      editCO2Sensor(item);
    }
    //!< Edit a thermal sensor
    if(item->parent() == &loader_.thermalSensorsNode && column == 1)
    {
      editThermalSensor(item);
    }
    //!< Edit a sound sensor
    if(item->parent() == &loader_.soundSensorsNode && column == 1)
    {
      editSoundSensor(item);
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
    //!< Save a rfid  
    if(item->parent() == &loader_.rfidAntennasNode && column == 3)
    {
      saveRfidAntenna(item);
    }
    //!< Save a co2 sensor  
    if(item->parent() == &loader_.co2SensorsNode && column == 3)
    {
      saveCO2Sensor(item);
    }
    //!< Save a thermal sensor  
    if(item->parent() == &loader_.thermalSensorsNode && column == 3)
    {
      saveThermalSensor(item);
    }
    //!< Save a sound sensor  
    if(item->parent() == &loader_.soundSensorsNode && column == 3)
    {
      saveSoundSensor(item);
    }
    //!< Load a rfid  
    if(item->parent() == &loader_.rfidAntennasNode && column == 4)
    {
      loadRfidAntenna(item);
    }
    //!< Load a co2 sensor  
    if(item->parent() == &loader_.co2SensorsNode && column == 4)
    {
      loadCO2Sensor(item);
    }
    //!< Load a thermal sensor  
    if(item->parent() == &loader_.thermalSensorsNode && column == 4)
    {
      loadThermalSensor(item);
    }
    //!< Load a sound sensor  
    if(item->parent() == &loader_.soundSensorsNode && column == 4)
    {
      loadSoundSensor(item);
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
    //!< Add point clicked
    if(item == &loader_.robotInfoFootprint && column == 2)
    {
      addFootprintPoint();
    }  
    //!< Erase a footprint point
    if(item->parent() == &loader_.robotInfoFootprint && column == 2)
    {
      eraseFootprintPoint(item);
    } 
    //!< Edit a footprint point
    if(item->parent() == &loader_.robotInfoFootprint && column == 1)
    {
      editFootprintPoint(item);
    }  
  }

  /**
  @brief Adds a footprint point in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addFootprintPoint(geometry_msgs::Point pt)
  {
    QTreeWidgetItem  *new_point;
      
    new_point = new QTreeWidgetItem();
    
    new_point->setText(0,QString("[") + QString().setNum(pt.x) + QString(",") + 
      QString().setNum(pt.y) + QString("]"));
    new_point->setIcon(1,loader_.editIcon);
    new_point->setIcon(2,loader_.removeIcon);
    
    loader_.robotInfoFootprint.addChild(new_point);

    loader_.robotInfoFootprint.setExpanded(true);
    updateRobotPreview();
  }

  /**
  @brief Adds a footprint point in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addFootprintPoint(void)
  {
    geometry_msgs::Point pt;
    pt.x = 0;
    pt.y = 0;
    
    new_robot_msg_.footprint.points.push_back(pt);

    QTreeWidgetItem  *new_point;
      
    new_point = new QTreeWidgetItem();
    
    new_point->setText(0,QString("[0,0]"));
    new_point->setIcon(1,loader_.editIcon);
    new_point->setIcon(2,loader_.removeIcon);
    
    loader_.robotInfoFootprint.addChild(new_point);

    loader_.robotInfoFootprint.setExpanded(true);
    updateRobotPreview();
  }
  
  /**
  @brief Erases a footprint point in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::eraseFootprintPoint(QTreeWidgetItem *item)
  {
    
    for(unsigned int i = 0 ; i < loader_.robotInfoFootprint.childCount() ; i++)
    {
      if(loader_.robotInfoFootprint.child(i) == item)
      {
        delete item;
        new_robot_msg_.footprint.points.erase(
          new_robot_msg_.footprint.points.begin() + i);
      }
    }
    updateRobotPreview();
  }
  
  /**
  @brief Called when the update button of the footprint widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateFootprintPoint(void)
  {
    QString xstr = loader_.robotFootLoader.robotFootprintX->text();
    QString ystr = loader_.robotFootLoader.robotFootprintY->text();
    float x = xstr.toFloat();
    float y = ystr.toFloat();
    
    int index = -1;
    for(unsigned int i = 0 ; i < loader_.robotInfoFootprint.childCount() ; i++)
    {
      if(loader_.robotInfoFootprint.child(i) == current_footprint_point_)
      {
        index = i;
        break;
      }
    }
    if( index == -1 )
    {
      return;
    }
    
    new_robot_msg_.footprint.points[index].x = x;
    new_robot_msg_.footprint.points[index].y = y;
    
    current_footprint_point_->setText(0,QString("[") + xstr + QString(",") +
      ystr + QString("]"));

    loader_.robotFootLoader.hide();
    
    updateRobotPreview();
  }

  /**
  @brief Called when the update button of the kinematic model widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateKinematicModel(void)
  {
    QString model_lit = loader_.kinematicPropLoader.comboBox->currentText();
    int model_ind = loader_.kinematicPropLoader.comboBox->currentIndex();
   
    switch(model_ind){
      case 0:
        new_robot_msg_.kinematicModel.type = "ideal";
        break;
      case 1:
        new_robot_msg_.kinematicModel.type = "omni";
        break;
    }

    loader_.kinematicPropLoader.hide();
    loader_.kinematicNode.setText(0,"Kinematic model: " \
      + QString(new_robot_msg_.kinematicModel.type.c_str()));

    
    updateRobotPreview();
  }

  /**
  @brief Called when the refresh button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateFootprintPointOpen(void)
  {
    QString xstr = loader_.robotFootLoader.robotFootprintX->text();
    QString ystr = loader_.robotFootLoader.robotFootprintY->text();
    float x = xstr.toFloat();
    float y = ystr.toFloat();

    int index = -1;
    for(unsigned int i = 0 ; i < loader_.robotInfoFootprint.childCount() ; i++)
    {
      if(loader_.robotInfoFootprint.child(i) == current_footprint_point_)
      {
        index = i;
        break;
      }
    }
    if( index == -1 )
    {
      return;
    }
    
    new_robot_msg_.footprint.points[index].x = x;
    new_robot_msg_.footprint.points[index].y = y;
    
    current_footprint_point_->setText(0,QString("[") + xstr + QString(",") +
      ystr + QString("]"));

    updateRobotPreview();
  }

  /**
  @brief Adds a laser sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addLaser(void)
  {
    QString laserFrameId=QString("laser_") + 
      QString().setNum(++CRobotCreatorConnector::laser_number);
    
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
    orientation->setText(1,QString().setNum(lmsg.pose.theta));
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
      QString().setNum(++CRobotCreatorConnector::sonar_number);
    
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
    QString rfidFrameId=QString("rfid_reader_") + 
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
  @brief Adds a co2 sensor in the new robot 
  **/
  void CRobotCreatorConnector::addCO2Sensor(void)
  {
    QString co2SensorFrameId=QString("co2_sensor_") + 
      QString().setNum(CRobotCreatorConnector::co2_sensors_number++);
    
    stdr_msgs::CO2SensorMsg smsg;
    smsg.frame_id = co2SensorFrameId.toStdString();
    smsg.maxRange = 3.0;
    //~ smsg.angleSpan = 360.0;
    smsg.pose.x = 0;
    smsg.pose.y = 0;
    smsg.pose.theta = 0;
    //~ smsg.signalCutoff = 0;
    smsg.frequency = 10;
    
    new_robot_msg_.co2Sensors.push_back(smsg);
    
    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,co2SensorFrameId);
    snode->setIcon(1,loader_.editIcon);
    snode->setIcon(2,loader_.removeIcon);
    snode->setIcon(3,loader_.saveIcon);
    snode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      //~ *angleSpan,
      *orientation,
      *maxRange,
      *poseX,
      *poseY,
      //~ *signalCutoff,
      *frequency;
      
    //~ angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    //~ signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    //~ angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    //~ signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    //~ angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    //~ signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    //~ snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    //~ snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.co2SensorsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.co2SensorsNode.setExpanded(true);
    updateRobotPreview();
  }
  /**
  @brief Adds a thermal sensor in the new robot 
  **/
  void CRobotCreatorConnector::addThermalSensor(void)
  {
    QString thermalSensorFrameId=QString("thermal_sensor_") + 
      QString().setNum(CRobotCreatorConnector::thermal_sensors_number++);
    
    stdr_msgs::ThermalSensorMsg smsg;
    smsg.frame_id = thermalSensorFrameId.toStdString();
    smsg.maxRange = 3.0;
    smsg.angleSpan = 20.0;
    smsg.pose.x = 0;
    smsg.pose.y = 0;
    smsg.pose.theta = 0;
    //~ smsg.signalCutoff = 0;
    smsg.frequency = 10;
    
    new_robot_msg_.thermalSensors.push_back(smsg);
    
    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,thermalSensorFrameId);
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
      //~ *signalCutoff,
      *frequency;
      
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    //~ signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    //~ signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    //~ signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    //~ snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.thermalSensorsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.thermalSensorsNode.setExpanded(true);
    updateRobotPreview();
  }
  /**
  @brief Adds a sound sensor in the new robot 
  **/
  void CRobotCreatorConnector::addSoundSensor(void)
  {
    QString soundSensorFrameId=QString("sound_sensor_") + 
      QString().setNum(CRobotCreatorConnector::sound_sensors_number++);
    
    stdr_msgs::SoundSensorMsg smsg;
    smsg.frame_id = soundSensorFrameId.toStdString();
    smsg.maxRange = 3.0;
    smsg.angleSpan = 180.0;
    smsg.pose.x = 0;
    smsg.pose.y = 0;
    smsg.pose.theta = 0;
    //~ smsg.signalCutoff = 0;
    smsg.frequency = 10;
    
    new_robot_msg_.soundSensors.push_back(smsg);
    
    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,soundSensorFrameId);
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
      //~ *signalCutoff,
      *frequency;
      
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    //~ signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    //~ signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    //~ signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    //~ snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.soundSensorsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.soundSensorsNode.setExpanded(true);
    updateRobotPreview();
  }
  
  /**
  @brief Adds an rfid antenna sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addRfidAntenna(stdr_msgs::RfidSensorMsg smsg)
  {
    CRobotCreatorConnector::rfid_number++;
    QString rfidFrameId=QString(smsg.frame_id.c_str());

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
  @brief Adds a co2 sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addCO2Sensor(stdr_msgs::CO2SensorMsg smsg)
  {
    CRobotCreatorConnector::co2_sensors_number++;
    QString co2SensorFrameId=QString(smsg.frame_id.c_str());

    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,co2SensorFrameId);
    snode->setIcon(1,loader_.editIcon);
    snode->setIcon(2,loader_.removeIcon);
    snode->setIcon(3,loader_.saveIcon);
    snode->setIcon(4,loader_.loadIcon);

    QTreeWidgetItem 
      //~ *angleSpan,
      *orientation,
      *maxRange,
      *poseX,
      *poseY,
      //~ *signalCutoff,
      *frequency;
      
    //~ angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    //~ signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    //~ angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    //~ signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    //~ angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    //~ signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    //~ snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    //~ snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.co2SensorsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.co2SensorsNode.setExpanded(true);
    updateRobotPreview();
  }
  /**
  @brief Adds a thermal sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addThermalSensor(stdr_msgs::ThermalSensorMsg smsg)
  {
    CRobotCreatorConnector::thermal_sensors_number++;
    QString thermalSensorFrameId=QString(smsg.frame_id.c_str());

    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,thermalSensorFrameId);
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
      //~ *signalCutoff,
      *frequency;
      
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    //~ signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    //~ signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    //~ signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    //~ snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.thermalSensorsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.thermalSensorsNode.setExpanded(true);
    updateRobotPreview();
  }
  /**
  @brief Adds a sound sensor in the new robot 
  @return void
  **/
  void CRobotCreatorConnector::addSoundSensor(stdr_msgs::SoundSensorMsg smsg)
  {
    CRobotCreatorConnector::sound_sensors_number++;
    QString soundSensorFrameId=QString(smsg.frame_id.c_str());

    QTreeWidgetItem  *snode;
    snode = new QTreeWidgetItem();
    snode->setText(0,soundSensorFrameId);
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
      //~ *signalCutoff,
      *frequency;
      
    angleSpan = new QTreeWidgetItem();
    orientation = new QTreeWidgetItem();
    maxRange = new QTreeWidgetItem();
    poseX = new QTreeWidgetItem();
    poseY = new QTreeWidgetItem();
    //~ signalCutoff = new QTreeWidgetItem();
    frequency = new QTreeWidgetItem();
    
    angleSpan->setText(0,QString("Angle span"));
    orientation->setText(0,QString("Orientation"));
    maxRange->setText(0,QString("Max range"));
    poseX->setText(0,QString("Pose - x"));
    poseY->setText(0,QString("Pose - y"));
    //~ signalCutoff->setText(0,QString("Signal cutoff"));
    frequency->setText(0,QString("Frequency"));
    
    angleSpan->setText(1,QString().setNum(smsg.angleSpan));
    orientation->setText(1,QString().setNum(smsg.pose.theta));
    maxRange->setText(1,QString().setNum(smsg.maxRange));
    poseX->setText(1,QString().setNum(smsg.pose.x));
    poseY->setText(1,QString().setNum(smsg.pose.y));
    //~ signalCutoff->setText(1,QString().setNum(smsg.signalCutoff));
    frequency->setText(1,QString().setNum(smsg.frequency));
    
    snode->addChild(angleSpan);
    snode->addChild(orientation);
    snode->addChild(maxRange);
    snode->addChild(poseX);
    snode->addChild(poseY);
    //~ snode->addChild(signalCutoff);
    snode->addChild(frequency);
    
    loader_.soundSensorsNode.addChild(snode);
    
    snode->setExpanded(false);
    loader_.soundSensorsNode.setExpanded(true);
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
  @brief Erases a specific co2 sensor based on a tree item
  **/
  void CRobotCreatorConnector::eraseCO2Sensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchCO2Sensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }
    new_robot_msg_.co2Sensors.erase(
      new_robot_msg_.co2Sensors.begin() + frameId);
    deleteTreeNode(item);
    updateRobotPreview();
  }
  /**
  @brief Erases a specific thermal sensor based on a tree item
  **/
  void CRobotCreatorConnector::eraseThermalSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchThermalSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }
    new_robot_msg_.thermalSensors.erase(
      new_robot_msg_.thermalSensors.begin() + frameId);
    deleteTreeNode(item);
    updateRobotPreview();
  }
  /**
  @brief Erases a specific sound sensor based on a tree item
  **/
  void CRobotCreatorConnector::eraseSoundSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchSoundSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }
    new_robot_msg_.soundSensors.erase(
      new_robot_msg_.soundSensors.begin() + frameId);
    deleteTreeNode(item);
    updateRobotPreview();
  }
  
  /**
  @brief Edits a specific footprint point based on a tree item. Initiates the footprint editor widget
  @param item [QTreeWidgetItem*] Tree item that holds the specific footprint point 
  @return void
  **/
  void CRobotCreatorConnector::editFootprintPoint(QTreeWidgetItem *item)
  {
    int index = -1;
    for(unsigned int i = 0 ; i < loader_.robotInfoFootprint.childCount() ; i++)
    {
      if(loader_.robotInfoFootprint.child(i) == item)
      {
        index = i;
        break;
      }
    }
    if( index == -1 )
    {
      return;
    }
    
    loader_.robotFootLoader.robotFootprintX->setText(
      QString().setNum(new_robot_msg_.footprint.points[index].x));
    loader_.robotFootLoader.robotFootprintY->setText(
      QString().setNum(new_robot_msg_.footprint.points[index].y));
    
    loader_.robotFootLoader.setWindowTitle(
      QApplication::translate(
        "Footprint point", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_footprint_point_ = item;
    
    loader_.robotFootLoader.show();
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
      stdr_parser::Parser::saveMessage(
        stdr_gui_tools::fixLaserAnglesToRad(lmsg), file_name_str);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
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
        tr("Resource files (*.yaml *.xml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::SonarSensorMsg smsg = new_robot_msg_.sonarSensors[sonarFrameId];

    try {
      stdr_parser::Parser::saveMessage(
        stdr_gui_tools::fixSonarAnglesToRad(smsg), file_name_str);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
  }
  
  /**
  @brief Saves a specific rfid reader sensor in a file
  @param item [QTreeWidgetItem*] The tree item that holds the sensor 
  @return void
  **/
  void CRobotCreatorConnector::saveRfidAntenna(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchRfid(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save RFID reader sensor"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Resource files (*.yaml *.xml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::RfidSensorMsg smsg = new_robot_msg_.rfidSensors[frameId];

    try {
      stdr_parser::Parser::saveMessage(
        stdr_gui_tools::fixRfidAnglesToRad(smsg), file_name_str);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
  }
  
  /**
  @brief Saves a specific co2 sensor in a file
  **/
  void CRobotCreatorConnector::saveCO2Sensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchCO2Sensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save CO2 sensor"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Resource files (*.yaml *.xml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::CO2SensorMsg smsg = new_robot_msg_.co2Sensors[frameId];

    try {
      stdr_parser::Parser::saveMessage(
        stdr_gui_tools::fixCO2AnglesToRad(smsg), file_name_str);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
  }
  /**
  @brief Saves a specific thermal sensor in a file
  **/
  void CRobotCreatorConnector::saveThermalSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchThermalSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save Thermal sensor"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Resource files (*.yaml *.xml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::ThermalSensorMsg smsg = new_robot_msg_.thermalSensors[frameId];

    try {
      stdr_parser::Parser::saveMessage(
        stdr_gui_tools::fixThermalAnglesToRad(smsg), file_name_str);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
  }
  /**
  @brief Saves a specific sound sensor in a file
  **/
  void CRobotCreatorConnector::saveSoundSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchSoundSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getSaveFileName(&loader_, 
      tr("Save Sound sensor"),
        QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"),
        tr("Resource files (*.yaml *.xml)"));
    
    std::string file_name_str=file_name.toStdString();
    stdr_msgs::SoundSensorMsg smsg = new_robot_msg_.soundSensors[frameId];

    try {
      stdr_parser::Parser::saveMessage(
        stdr_gui_tools::fixSoundAnglesToRad(smsg), file_name_str);
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
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
        tr("Resource Files (*.yaml *.xml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    std::string old_frame_id = item->text(0).toStdString();
    stdr_msgs::LaserSensorMsg lmsg;
    try {
      lmsg = 
        stdr_parser::Parser::createMessage<stdr_msgs::LaserSensorMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
    lmsg = stdr_gui_tools::fixLaserAnglesToDegrees(lmsg);
    lmsg.frame_id = old_frame_id;
    new_robot_msg_.laserSensors[laserFrameId]=lmsg;
    updateLaserTree(item,lmsg);
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
        tr("Resource Files (*.yaml *.xml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    stdr_msgs::SonarSensorMsg smsg;
    std::string old_frame_id = item->text(0).toStdString();
    try {
      smsg = 
        stdr_parser::Parser::createMessage<stdr_msgs::SonarSensorMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
    smsg = stdr_gui_tools::fixSonarAnglesToDegrees(smsg);
    smsg.frame_id = old_frame_id;
    new_robot_msg_.sonarSensors[sonarFrameId]=smsg;
    updateSonarTree(item,smsg);
    updateRobotPreview(); 
  }
  
  /**
  @brief Loads a specific rfid antenna sensor from a file
  @param item [QTreeWidgetItem*] The tree item that holds the sensor 
  @return void
  **/
  void CRobotCreatorConnector::loadRfidAntenna(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchRfid(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load RFID reader sensor"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Resource Files (*.yaml *.xml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    stdr_msgs::RfidSensorMsg smsg;
    std::string old_frame_id = item->text(0).toStdString();
    try {
      smsg = 
        stdr_parser::Parser::createMessage<stdr_msgs::RfidSensorMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
    smsg = stdr_gui_tools::fixRfidAnglesToDegrees(smsg);
    smsg.frame_id = old_frame_id;
    new_robot_msg_.rfidSensors[frameId]=smsg;
    updateRfidTree(item,smsg);
    updateRobotPreview(); 
  }
  /**
  @brief Loads a specific co2 sensor from a file
  **/
  void CRobotCreatorConnector::loadCO2Sensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchCO2Sensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load CO2 sensor"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Resource Files (*.yaml *.xml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    stdr_msgs::CO2SensorMsg smsg;
    std::string old_frame_id = item->text(0).toStdString();
    try {
      smsg = 
        stdr_parser::Parser::createMessage<stdr_msgs::CO2SensorMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
    smsg = stdr_gui_tools::fixCO2AnglesToDegrees(smsg);
    smsg.frame_id = old_frame_id;
    new_robot_msg_.co2Sensors[frameId]=smsg;
    updateCO2SensorTree(item,smsg);
    updateRobotPreview(); 
  }
  /**
  @brief Loads a specific thermal sensor from a file
  **/
  void CRobotCreatorConnector::loadThermalSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchThermalSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load thermal sensor"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Resource Files (*.yaml *.xml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    stdr_msgs::ThermalSensorMsg smsg;
    std::string old_frame_id = item->text(0).toStdString();
    try {
      smsg = 
        stdr_parser::Parser::createMessage<stdr_msgs::ThermalSensorMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
    smsg = stdr_gui_tools::fixThermalAnglesToDegrees(smsg);
    smsg.frame_id = old_frame_id;
    new_robot_msg_.thermalSensors[frameId]=smsg;
    updateThermalSensorTree(item,smsg);
    updateRobotPreview(); 
  }
  /**
  @brief Loads a specific sound sensor from a file
  **/
  void CRobotCreatorConnector::loadSoundSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchSoundSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    QString file_name = QFileDialog::getOpenFileName(
      &loader_,
      tr("Load sound sensor"), 
      QString().fromStdString(
        stdr_gui_tools::getRosPackagePath("stdr_resources")) + 
        QString("/resources/"), 
        tr("Resource Files (*.yaml *.xml)"));
    
    if (file_name.isEmpty()) {
      return;
    }
    stdr_msgs::SoundSensorMsg smsg;
    std::string old_frame_id = item->text(0).toStdString();
    try {
      smsg = 
        stdr_parser::Parser::createMessage<stdr_msgs::SoundSensorMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }
    smsg = stdr_gui_tools::fixSoundAnglesToDegrees(smsg);
    smsg.frame_id = old_frame_id;
    new_robot_msg_.soundSensors[frameId]=smsg;
    updateSoundSensorTree(item,smsg);
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
  @brief Edits a specific co2 sensor based on a tree item. \
  Initiates the co2 sensor editor widget
  **/
  void CRobotCreatorConnector::editCO2Sensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchCO2Sensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    loader_.co2SensorPropLoader.maxDistance->setText(
      QString().setNum(new_robot_msg_.co2Sensors[frameId].maxRange));
  
    loader_.co2SensorPropLoader.x_->setText(
      QString().setNum(new_robot_msg_.co2Sensors[frameId].pose.x));
  
    loader_.co2SensorPropLoader.y_->setText(
      QString().setNum(new_robot_msg_.co2Sensors[frameId].pose.y));
  
    //~ loader_.co2SensorPropLoader.angleSpan->setText(
      //~ QString().setNum(new_robot_msg_.co2Sensors[frameId].angleSpan));
  
    loader_.co2SensorPropLoader.orientation->setText(
      QString().setNum(new_robot_msg_.co2Sensors[frameId].pose.theta));
  
    //~ loader_.co2SensorPropLoader.signalCutoff->setText(
      //~ QString().setNum(new_robot_msg_.co2Sensors[frameId].signalCutoff));
  
    loader_.co2SensorPropLoader.frequency->setText(
      QString().setNum(new_robot_msg_.co2Sensors[frameId].frequency));
    
    loader_.co2SensorPropLoader.setWindowTitle(
      QApplication::translate(
        "CO2SensorProperties", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_co2_sensor_ = item;
    
    loader_.co2SensorPropLoader.show();
  }
  /**
  @brief Edits a specific thermal sensor based on a tree item. \
  Initiates the thermal sensor editor widget
  **/
  void CRobotCreatorConnector::editThermalSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchThermalSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    loader_.thermalSensorPropLoader.maxDistance->setText(
      QString().setNum(new_robot_msg_.thermalSensors[frameId].maxRange));
  
    loader_.thermalSensorPropLoader.x_->setText(
      QString().setNum(new_robot_msg_.thermalSensors[frameId].pose.x));
  
    loader_.thermalSensorPropLoader.y_->setText(
      QString().setNum(new_robot_msg_.thermalSensors[frameId].pose.y));
  
    loader_.thermalSensorPropLoader.angleSpan->setText(
      QString().setNum(new_robot_msg_.thermalSensors[frameId].angleSpan));
  
    loader_.thermalSensorPropLoader.orientation->setText(
      QString().setNum(new_robot_msg_.thermalSensors[frameId].pose.theta));

    loader_.thermalSensorPropLoader.frequency->setText(
      QString().setNum(new_robot_msg_.thermalSensors[frameId].frequency));
    
    loader_.thermalSensorPropLoader.setWindowTitle(
      QApplication::translate(
        "ThermalSensorProperties", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_thermal_sensor_ = item;
    
    loader_.thermalSensorPropLoader.show();
  }
  /**
  @brief Edits a specific sound sensor based on a tree item. \
  Initiates the sound sensor editor widget
  **/
  void CRobotCreatorConnector::editSoundSensor(QTreeWidgetItem *item)
  {
    unsigned int frameId = searchSoundSensor(item->text(0));
    if(frameId == -1) 
    {
      return;
    }  
    loader_.soundSensorPropLoader.maxDistance->setText(
      QString().setNum(new_robot_msg_.soundSensors[frameId].maxRange));
  
    loader_.soundSensorPropLoader.x_->setText(
      QString().setNum(new_robot_msg_.soundSensors[frameId].pose.x));
  
    loader_.soundSensorPropLoader.y_->setText(
      QString().setNum(new_robot_msg_.soundSensors[frameId].pose.y));
  
    loader_.soundSensorPropLoader.angleSpan->setText(
      QString().setNum(new_robot_msg_.soundSensors[frameId].angleSpan));
  
    loader_.soundSensorPropLoader.orientation->setText(
      QString().setNum(new_robot_msg_.soundSensors[frameId].pose.theta));

    loader_.soundSensorPropLoader.frequency->setText(
      QString().setNum(new_robot_msg_.soundSensors[frameId].frequency));
    
    loader_.soundSensorPropLoader.setWindowTitle(
      QApplication::translate(
        "SoundSensorProperties", 
        item->text(0).toStdString().c_str(), 
        0, 
        QApplication::UnicodeUTF8));
    
    current_sound_sensor_ = item;
    
    loader_.soundSensorPropLoader.show();
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
  @brief Returns the ID of an co2 sensor
  @param frameId [QString] The frame id of the co2 sensor 
  @return int
  **/
  int CRobotCreatorConnector::searchCO2Sensor(QString frameId)
  {
    for(unsigned int i = 0 ; i < new_robot_msg_.co2Sensors.size() ; i++)
    {
      if(new_robot_msg_.co2Sensors[i].frame_id == frameId.toStdString())
      {
        return i;
      }
    }
    return -1;
  }
  /**
  @brief Returns the ID of a thermal sensor
  @param frameId [QString] The frame id of the thermal sensor 
  @return int
  **/
  int CRobotCreatorConnector::searchThermalSensor(QString frameId)
  {
    for(unsigned int i = 0 ; i < new_robot_msg_.thermalSensors.size() ; i++)
    {
      if(new_robot_msg_.thermalSensors[i].frame_id == frameId.toStdString())
      {
        return i;
      }
    }
    return -1;
  }
  /**
  @brief Returns the ID of a sound sensor
  @param frameId [QString] The frame id of the sound sensor 
  @return int
  **/
  int CRobotCreatorConnector::searchSoundSensor(QString frameId)
  {
    for(unsigned int i = 0 ; i < new_robot_msg_.soundSensors.size() ; i++)
    {
      if(new_robot_msg_.soundSensors[i].frame_id == frameId.toStdString())
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
      if(loader_.robotNode.child(i)->text(0) == QString("Radius"))
      {
        loader_.robotNode.child(i)->
          setText(1,QString().setNum(new_robot_msg_.footprint.radius));
      }
    }
    int count = 0;
    
    count = loader_.lasersNode.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      QTreeWidgetItem *child = loader_.lasersNode.child(i);
      loader_.lasersNode.removeChild(loader_.lasersNode.child(i));
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.laserSensors.size() ; i++)
    {
      addLaser(new_robot_msg_.laserSensors[i]);
    }
    
    count = loader_.rfidAntennasNode.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      QTreeWidgetItem *child = loader_.rfidAntennasNode.child(i);
      loader_.rfidAntennasNode.removeChild(loader_.rfidAntennasNode.child(i));
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.rfidSensors.size() ; i++)
    {
      addRfidAntenna(new_robot_msg_.rfidSensors[i]);
    }
    
    count = loader_.thermalSensorsNode.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      QTreeWidgetItem *child = loader_.thermalSensorsNode.child(i);
      loader_.thermalSensorsNode.removeChild(loader_.thermalSensorsNode.child(i));
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.thermalSensors.size() ; i++)
    {
      addThermalSensor(new_robot_msg_.thermalSensors[i]);
    }
    
    count = loader_.soundSensorsNode.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      QTreeWidgetItem *child = loader_.soundSensorsNode.child(i);
      loader_.soundSensorsNode.removeChild(loader_.soundSensorsNode.child(i));
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.soundSensors.size() ; i++)
    {
      addSoundSensor(new_robot_msg_.soundSensors[i]);
    }
    
    count = loader_.sonarsNode.childCount();
    for(int i = count - 1 ; i >= 0 ; i--)
    {
      QTreeWidgetItem *child = loader_.sonarsNode.child(i);
      loader_.sonarsNode.removeChild(loader_.sonarsNode.child(i));
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.sonarSensors.size() ; i++)
    {
      addSonar(new_robot_msg_.sonarSensors[i]);
    }
    
    for(unsigned int i = 0 ; i < new_robot_msg_.footprint.points.size() ; i++)
    {
      addFootprintPoint(new_robot_msg_.footprint.points[i]);
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
        //~ new_robot_msg_.laserSensors[laserFrameId].minAngle += orientation;
        //~ new_robot_msg_.laserSensors[laserFrameId].maxAngle += orientation;
        new_robot_msg_.laserSensors[laserFrameId].pose.theta = orientation;
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
  @brief Called when the refresh button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateLaserOpen(void)
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
        //~ new_robot_msg_.laserSensors[laserFrameId].minAngle += orientation;
        //~ new_robot_msg_.laserSensors[laserFrameId].maxAngle += orientation;
        new_robot_msg_.laserSensors[laserFrameId].pose.theta = orientation;
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
  @brief Called when the refresh button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateSonarOpen(void)
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
  @brief Updates a tree item with a specific rfid antenna sensor
  @param item [QTreeWidgetItem*] The tree item that will be updated
  @param l [stdr_msgs::RfidSensorMsg] The rfid antenna sensor message
  @return void
  **/
  void CRobotCreatorConnector::updateRfidTree(
    QTreeWidgetItem *item,
    stdr_msgs::RfidSensorMsg l)
  {
    unsigned int frameId=searchRfid(item->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < item->childCount() ; i++)
    {
      if(item->child(i)->text(0) == QString("Angle span"))
      {
        item->child(i)->setText(1,QString().setNum(l.angleSpan));
      }
      else if(item->child(i)->text(0) == QString("Orientation"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.theta));
      }
      else if(item->child(i)->text(0) == QString("Max range"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxRange));
      }
      else if(item->child(i)->text(0) == QString("Signal cutoff"))
      {
        item->child(i)->setText(1,QString().setNum(l.signalCutoff));
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
  @brief Updates a tree item with a specific co2 sensor
  **/
  void CRobotCreatorConnector::updateCO2SensorTree(
    QTreeWidgetItem *item,
    stdr_msgs::CO2SensorMsg l)
  {
    unsigned int frameId=searchCO2Sensor(item->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < item->childCount() ; i++)
    {
      //~ if(item->child(i)->text(0) == QString("Angle span"))
      //~ {
        //~ item->child(i)->setText(1,QString().setNum(l.angleSpan));
      //~ }
      if(item->child(i)->text(0) == QString("Orientation"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.theta));
      }
      else if(item->child(i)->text(0) == QString("Max range"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxRange));
      }
      //~ else if(item->child(i)->text(0) == QString("Signal cutoff"))
      //~ {
        //~ item->child(i)->setText(1,QString().setNum(l.signalCutoff));
      //~ }
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
  @brief Updates a tree item with a specific thermal sensor
  **/
  void CRobotCreatorConnector::updateThermalSensorTree(
    QTreeWidgetItem *item,
    stdr_msgs::ThermalSensorMsg l)
  {
    unsigned int frameId=searchThermalSensor(item->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < item->childCount() ; i++)
    {
      if(item->child(i)->text(0) == QString("Angle span"))
      {
        item->child(i)->setText(1,QString().setNum(l.angleSpan));
      }
      else if(item->child(i)->text(0) == QString("Orientation"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.theta));
      }
      else if(item->child(i)->text(0) == QString("Max range"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxRange));
      }
      //~ else if(item->child(i)->text(0) == QString("Signal cutoff"))
      //~ {
        //~ item->child(i)->setText(1,QString().setNum(l.signalCutoff));
      //~ }
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
  @brief Updates a tree item with a specific sound sensor
  **/
  void CRobotCreatorConnector::updateSoundSensorTree(
    QTreeWidgetItem *item,
    stdr_msgs::SoundSensorMsg l)
  {
    unsigned int frameId=searchSoundSensor(item->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < item->childCount() ; i++)
    {
      if(item->child(i)->text(0) == QString("Angle span"))
      {
        item->child(i)->setText(1,QString().setNum(l.angleSpan));
      }
      else if(item->child(i)->text(0) == QString("Orientation"))
      {
        item->child(i)->setText(1,QString().setNum(l.pose.theta));
      }
      else if(item->child(i)->text(0) == QString("Max range"))
      {
        item->child(i)->setText(1,QString().setNum(l.maxRange));
      }
      //~ else if(item->child(i)->text(0) == QString("Signal cutoff"))
      //~ {
        //~ item->child(i)->setText(1,QString().setNum(l.signalCutoff));
      //~ }
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
  void CRobotCreatorConnector::updateRfidAntenna(void)
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
  @brief Called when the update button of the properties widget is clicked 
  **/ 
  void CRobotCreatorConnector::updateCO2Sensor(void)
  {
    unsigned int frameId = searchCO2Sensor(current_co2_sensor_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_co2_sensor_->childCount() ; i++)
    {
      
      //!< Rfid antenna angle span
      //~ if(current_rfid_->child(i)->text(0) == QString("Angle span"))
      //~ {
        //~ float angle_span = 
          //~ loader_.rfidAntennaPropLoader.rfidAngleSpan->text().toFloat();
        //~ if( angle_span <= 0 )
        //~ {
          //~ showMessage(QString("Rfid antenna angle span invalid :") + 
            //~ QString().setNum(angle_span));
          //~ return;
        //~ }
        //~ current_rfid_->child(i)->setText(1,QString().setNum(angle_span));
        //~ new_robot_msg_.rfidSensors[frameId].angleSpan = angle_span;
      //~ }
      
      //!< Orientation
      if(current_co2_sensor_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.co2SensorPropLoader.orientation->
            text().toFloat();
        current_co2_sensor_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.co2Sensors[frameId].pose.theta = orientation;
      }
      
      //!< Rfid antenna max range
      else if(current_co2_sensor_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.co2SensorPropLoader.maxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("CO2 sensor maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_co2_sensor_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.co2Sensors[frameId].maxRange = maxRange;
      }
      
      //!< Sensor pose - x coordinate
      else if(current_co2_sensor_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.co2SensorPropLoader.x_->text().toFloat();
        current_co2_sensor_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.co2Sensors[frameId].pose.x = dx;
      }
      
      //!< Sensor pose - y coordinate
      else if(current_co2_sensor_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.co2SensorPropLoader.y_->text().toFloat();
        current_co2_sensor_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.co2Sensors[frameId].pose.y = dy;
      }

      //!< Publishing frequency
      else if(current_co2_sensor_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.co2SensorPropLoader.frequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("CO2 sensor publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_co2_sensor_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.co2Sensors[frameId].frequency = frequency;
      }
    }

    loader_.co2SensorPropLoader.hide();
    
    updateRobotPreview();
  }
  /**
  @brief Called when the update button of the properties widget is clicked 
  **/ 
  void CRobotCreatorConnector::updateThermalSensor(void)
  {
    unsigned int frameId = searchThermalSensor(current_thermal_sensor_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_thermal_sensor_->childCount() ; i++)
    {
      
      //~ Sensor angle span
      if(current_thermal_sensor_->child(i)->text(0) == QString("Angle span"))
      {
        float angle_span = 
          loader_.thermalSensorPropLoader.angleSpan->text().toFloat();
        if( angle_span <= 0 )
        {
          showMessage(QString("Thermal sensor angle span invalid :") + 
            QString().setNum(angle_span));
          return;
        }
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(angle_span));
        new_robot_msg_.thermalSensors[frameId].angleSpan = angle_span;
      }
      
      //!< Orientation
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.thermalSensorPropLoader.orientation->
            text().toFloat();
        current_thermal_sensor_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.thermalSensors[frameId].pose.theta = orientation;
      }
      
      //!< Rfid antenna max range
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.thermalSensorPropLoader.maxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("Thermal sensor maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.thermalSensors[frameId].maxRange = maxRange;
      }
      
      //!< Sensor pose - x coordinate
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.thermalSensorPropLoader.x_->text().toFloat();
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.thermalSensors[frameId].pose.x = dx;
      }
      
      //!< Sensor pose - y coordinate
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.thermalSensorPropLoader.y_->text().toFloat();
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.thermalSensors[frameId].pose.y = dy;
      }

      //!< Publishing frequency
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.thermalSensorPropLoader.frequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Thermal sensor publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.thermalSensors[frameId].frequency = frequency;
      }
    }

    loader_.thermalSensorPropLoader.hide();
    
    updateRobotPreview();
  }
  /**
  @brief Called when the update button of the properties widget is clicked 
  **/ 
  void CRobotCreatorConnector::updateSoundSensor(void)
  {
    unsigned int frameId = searchSoundSensor(current_sound_sensor_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_sound_sensor_->childCount() ; i++)
    {
      
      //~ Sensor angle span
      if(current_sound_sensor_->child(i)->text(0) == QString("Angle span"))
      {
        float angle_span = 
          loader_.soundSensorPropLoader.angleSpan->text().toFloat();
        if( angle_span <= 0 )
        {
          showMessage(QString("Sound sensor angle span invalid :") + 
            QString().setNum(angle_span));
          return;
        }
        current_sound_sensor_->child(i)->setText(1,QString().setNum(angle_span));
        new_robot_msg_.soundSensors[frameId].angleSpan = angle_span;
      }
      
      //!< Orientation
      else if(current_sound_sensor_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.soundSensorPropLoader.orientation->
            text().toFloat();
        current_sound_sensor_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.soundSensors[frameId].pose.theta = orientation;
      }
      
      //!< Rfid antenna max range
      else if(current_sound_sensor_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.soundSensorPropLoader.maxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("Sound sensor maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_sound_sensor_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.soundSensors[frameId].maxRange = maxRange;
      }
      
      //!< Sensor pose - x coordinate
      else if(current_sound_sensor_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.soundSensorPropLoader.x_->text().toFloat();
        current_sound_sensor_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.soundSensors[frameId].pose.x = dx;
      }
      
      //!< Sensor pose - y coordinate
      else if(current_sound_sensor_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.soundSensorPropLoader.y_->text().toFloat();
        current_sound_sensor_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.soundSensors[frameId].pose.y = dy;
      }

      //!< Publishing frequency
      else if(current_sound_sensor_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.soundSensorPropLoader.frequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Sound sensor publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_sound_sensor_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.soundSensors[frameId].frequency = frequency;
      }
    }

    loader_.soundSensorPropLoader.hide();
    
    updateRobotPreview();
  }
  
  /**
  @brief Called when the refresh button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateRfidAntennaOpen(void)
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
    
    updateRobotPreview();
  }
  /**
  @brief Called when the refresh button of the properties widget is clicked 
  **/ 
  void CRobotCreatorConnector::updateCO2SensorOpen(void)
  {
    unsigned int frameId = searchCO2Sensor(current_co2_sensor_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_co2_sensor_->childCount() ; i++)
    {
      
      //~ //!< Angle span
      //~ if(current_co2_sensor_->child(i)->text(0) == QString("Angle span"))
      //~ {
        //~ float angle_span = 
          //~ loader_.co2SensorPropLoader.angleSpan->text().toFloat();
        //~ if( angle_span <= 0 )
        //~ {
          //~ showMessage(QString("Rfid antenna angle span invalid :") + 
            //~ QString().setNum(angle_span));
          //~ return;
        //~ }
        //~ current_rfid_->child(i)->setText(1,QString().setNum(angle_span));
        //~ new_robot_msg_.rfidSensors[frameId].angleSpan = angle_span;
      //~ }
      
      //!< Rfid antenna orientation
      if(current_co2_sensor_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.co2SensorPropLoader.orientation->
            text().toFloat();
        current_co2_sensor_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.co2Sensors[frameId].pose.theta = orientation;
      }
      
      //!< Max range
      else if(current_co2_sensor_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.co2SensorPropLoader.maxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("CO2 source maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_co2_sensor_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.co2Sensors[frameId].maxRange = maxRange;
      }
      
      //!< Pose - x coordinate
      else if(current_co2_sensor_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.co2SensorPropLoader.x_->text().toFloat();
        current_co2_sensor_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.co2Sensors[frameId].pose.x = dx;
      }
      
      //!< Pose - y coordinate
      else if(current_co2_sensor_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.co2SensorPropLoader.y_->text().toFloat();
        current_co2_sensor_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.co2Sensors[frameId].pose.y = dy;
      }
      
      //!< Publishing frequency
      else if(current_co2_sensor_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.co2SensorPropLoader.frequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("CO2 sensor publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_co2_sensor_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.co2Sensors[frameId].frequency = frequency;
      }
    }
    
    updateRobotPreview();
  }
  /**
  @brief Called when the refresh button of the properties widget is clicked 
  **/ 
  void CRobotCreatorConnector::updateThermalSensorOpen(void)
  {
    unsigned int frameId = searchThermalSensor(current_thermal_sensor_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_thermal_sensor_->childCount() ; i++)
    {
      
      //!< Angle span
      if(current_thermal_sensor_->child(i)->text(0) == QString("Angle span"))
      {
        float angle_span = 
          loader_.thermalSensorPropLoader.angleSpan->text().toFloat();
        if( angle_span <= 0 )
        {
          showMessage(QString("Thermal sensor angle span invalid :") + 
            QString().setNum(angle_span));
          return;
        }
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(angle_span));
        new_robot_msg_.thermalSensors[frameId].angleSpan = angle_span;
      }
      
      //!< Orientation
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.thermalSensorPropLoader.orientation->
            text().toFloat();
        current_thermal_sensor_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.thermalSensors[frameId].pose.theta = orientation;
      }
      
      //!< Max range
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.thermalSensorPropLoader.maxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("Thermal sensor maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.thermalSensors[frameId].maxRange = maxRange;
      }
      
      //!< Pose - x coordinate
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.thermalSensorPropLoader.x_->text().toFloat();
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.thermalSensors[frameId].pose.x = dx;
      }
      
      //!< Pose - y coordinate
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.thermalSensorPropLoader.y_->text().toFloat();
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.thermalSensors[frameId].pose.y = dy;
      }
      
      //!< Publishing frequency
      else if(current_thermal_sensor_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.thermalSensorPropLoader.frequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Thermal sensor publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_thermal_sensor_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.thermalSensors[frameId].frequency = frequency;
      }
    }
    
    updateRobotPreview();
  }
  /**
  @brief Called when the refresh button of the properties widget is clicked 
  **/ 
  void CRobotCreatorConnector::updateSoundSensorOpen(void)
  {
    unsigned int frameId = searchSoundSensor(current_sound_sensor_->text(0));
    if(frameId == -1)
    {
      return;
    }
    for(unsigned int i = 0 ; i < current_sound_sensor_->childCount() ; i++)
    {
      
      //!< Angle span
      if(current_sound_sensor_->child(i)->text(0) == QString("Angle span"))
      {
        float angle_span = 
          loader_.soundSensorPropLoader.angleSpan->text().toFloat();
        if( angle_span <= 0 )
        {
          showMessage(QString("Sound sensor angle span invalid :") + 
            QString().setNum(angle_span));
          return;
        }
        current_sound_sensor_->child(i)->setText(1,QString().setNum(angle_span));
        new_robot_msg_.soundSensors[frameId].angleSpan = angle_span;
      }
      
      //!< Orientation
      else if(current_sound_sensor_->child(i)->text(0) == QString("Orientation"))
      {
        float orientation = 
          loader_.soundSensorPropLoader.orientation->
            text().toFloat();
        current_sound_sensor_->child(i)->
          setText(1,QString().setNum(orientation));
        new_robot_msg_.soundSensors[frameId].pose.theta = orientation;
      }
      
      //!< Max range
      else if(current_sound_sensor_->child(i)->text(0) == QString("Max range"))
      {
        float maxRange = 
          loader_.soundSensorPropLoader.maxDistance->
            text().toFloat();
        if( maxRange <= 0 )
        {
          showMessage(QString("Sound sensor maximum range invalid :") + 
            QString().setNum(maxRange));
          return;
        }
        current_sound_sensor_->child(i)->setText(1,QString().setNum(maxRange));
        new_robot_msg_.soundSensors[frameId].maxRange = maxRange;
      }
      
      //!< Pose - x coordinate
      else if(current_sound_sensor_->child(i)->text(0) == QString("Pose - x"))
      {
        float dx = loader_.soundSensorPropLoader.x_->text().toFloat();
        current_sound_sensor_->child(i)->setText(1,QString().setNum(dx));
        new_robot_msg_.soundSensors[frameId].pose.x = dx;
      }
      
      //!< Pose - y coordinate
      else if(current_sound_sensor_->child(i)->text(0) == QString("Pose - y"))
      {
        float dy = loader_.soundSensorPropLoader.y_->text().toFloat();
        current_sound_sensor_->child(i)->setText(1,QString().setNum(dy));
        new_robot_msg_.soundSensors[frameId].pose.y = dy;
      }
      
      //!< Publishing frequency
      else if(current_sound_sensor_->child(i)->text(0) == QString("Frequency"))
      {
        float frequency = 
          loader_.soundSensorPropLoader.frequency->
            text().toFloat();
        if( frequency <= 0 )
        {
          showMessage(QString("Sound sensor publishing frequency invalid :") + 
            QString().setNum(frequency));
          return;
        }
        current_sound_sensor_->child(i)->setText(1,QString().setNum(frequency));
        new_robot_msg_.soundSensors[frameId].frequency = frequency;
      }
    }
    
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
    loader_.robotPropLoader.robotRadius->
      setText(QString().setNum(new_robot_msg_.footprint.radius));
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
      if(loader_.robotNode.child(i)->text(0) == QString("Radius"))
      {
        loader_.robotNode.child(i)->
          setText(1,loader_.robotPropLoader.robotRadius->text());
        new_robot_msg_.footprint.radius = 
          loader_.robotPropLoader.robotRadius->text().toFloat();
      }
    }

    loader_.robotPropLoader.hide();
    
    updateRobotPreview();
  }
  
  /**
  @brief Called when the refresh button of the properties widget is clicked 
  @return void
  **/ 
  void CRobotCreatorConnector::updateRobotOpen(void)
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
      if(loader_.robotNode.child(i)->text(0) == QString("Radius"))
      {
        loader_.robotNode.child(i)->
          setText(1,loader_.robotPropLoader.robotRadius->text());
        new_robot_msg_.footprint.radius = 
          loader_.robotPropLoader.robotRadius->text().toFloat();
      }
    }
    
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
    for(unsigned int i = 0 ; i < new_robot_msg_.footprint.points.size() ; i++)
    {
      if(climax_ < new_robot_msg_.footprint.points[i].x)
      {
        climax_ = new_robot_msg_.footprint.points[i].x;
      }
      if(climax_ < new_robot_msg_.footprint.points[i].y)
      {
        climax_ = new_robot_msg_.footprint.points[i].y;
      }
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
    for(unsigned int i = 0 ; i < new_robot_msg_.co2Sensors.size() ; i++)
    {
      if(climax_ < (new_robot_msg_.co2Sensors[i].maxRange + 
          new_robot_msg_.co2Sensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.co2Sensors[i].maxRange + 
          new_robot_msg_.co2Sensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.co2Sensors[i].maxRange + 
          new_robot_msg_.co2Sensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.co2Sensors[i].maxRange + 
          new_robot_msg_.co2Sensors[i].pose.y;  
      }
      if(climax_ < (new_robot_msg_.co2Sensors[i].maxRange - 
          new_robot_msg_.co2Sensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.co2Sensors[i].maxRange - 
          new_robot_msg_.co2Sensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.co2Sensors[i].maxRange - 
          new_robot_msg_.co2Sensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.co2Sensors[i].maxRange - 
          new_robot_msg_.co2Sensors[i].pose.y;  
      }
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.thermalSensors.size() ; i++)
    {
      if(climax_ < (new_robot_msg_.thermalSensors[i].maxRange + 
          new_robot_msg_.thermalSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.thermalSensors[i].maxRange + 
          new_robot_msg_.thermalSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.thermalSensors[i].maxRange + 
          new_robot_msg_.thermalSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.thermalSensors[i].maxRange + 
          new_robot_msg_.thermalSensors[i].pose.y;  
      }
      if(climax_ < (new_robot_msg_.thermalSensors[i].maxRange - 
          new_robot_msg_.thermalSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.thermalSensors[i].maxRange - 
          new_robot_msg_.thermalSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.thermalSensors[i].maxRange - 
          new_robot_msg_.thermalSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.thermalSensors[i].maxRange - 
          new_robot_msg_.thermalSensors[i].pose.y;  
      }
    }
    for(unsigned int i = 0 ; i < new_robot_msg_.soundSensors.size() ; i++)
    {
      if(climax_ < (new_robot_msg_.soundSensors[i].maxRange + 
          new_robot_msg_.soundSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.soundSensors[i].maxRange + 
          new_robot_msg_.soundSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.soundSensors[i].maxRange + 
          new_robot_msg_.soundSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.soundSensors[i].maxRange + 
          new_robot_msg_.soundSensors[i].pose.y;  
      }
      if(climax_ < (new_robot_msg_.soundSensors[i].maxRange - 
          new_robot_msg_.soundSensors[i].pose.x) )
      {
        climax_ = new_robot_msg_.soundSensors[i].maxRange - 
          new_robot_msg_.soundSensors[i].pose.x;
      }
      if(climax_ < (new_robot_msg_.soundSensors[i].maxRange - 
          new_robot_msg_.soundSensors[i].pose.y) )
      {
        climax_ = new_robot_msg_.soundSensors[i].maxRange - 
          new_robot_msg_.soundSensors[i].pose.y;  
      }
    }
    
    climax_ = 230.0 / climax_;
    drawRobot();
    drawLasers();
    drawSonars();
    drawRfidAntennas();
    drawCO2Sensors();
    drawThermalSensors();
    drawSoundSensors();
    
    loader_.robotTreeWidget->resizeColumnToContents(0);
    loader_.robotTreeWidget->resizeColumnToContents(1);
    loader_.robotTreeWidget->resizeColumnToContents(2);
    loader_.robotTreeWidget->resizeColumnToContents(3);
    loader_.robotTreeWidget->resizeColumnToContents(4);
  }
  
  /**
  @brief Draws a circular robot
  @return void
  **/
  void CRobotCreatorConnector::drawRobot(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    painter.setPen(Qt::blue);
    
    if(new_robot_msg_.footprint.points.size() == 0)
    {
      painter.drawEllipse(
        250 - new_robot_msg_.footprint.radius * climax_,
        250 - new_robot_msg_.footprint.radius * climax_,
        new_robot_msg_.footprint.radius * climax_ * 2,
        new_robot_msg_.footprint.radius * climax_ * 2);
        
      painter.setPen(Qt::red);
      
      painter.drawLine(  
        250,
        250,
        250 + new_robot_msg_.footprint.radius * climax_ * 1.05 * cos(
          new_robot_msg_.initialPose.theta / 180.0 * STDR_PI),
        250 - new_robot_msg_.footprint.radius * climax_ * 1.05 * sin(
          new_robot_msg_.initialPose.theta / 180.0 * STDR_PI));
    }
    else
    {
      QPointF *points = new QPointF[new_robot_msg_.footprint.points.size() + 1];
      
      float max_val = 0;
      
      for(unsigned int i = 0 ; 
        i < new_robot_msg_.footprint.points.size() + 1 ; i++)
      {
        float x = new_robot_msg_.footprint.points
          [i % new_robot_msg_.footprint.points.size()].x;
        float y = - new_robot_msg_.footprint.points
          [i % new_robot_msg_.footprint.points.size()].y;
        
        if( pow(x,2) + pow(y,2) > max_val )
        {
          max_val = pow(x,2) + pow(y,2);
        }
        
        points[i] = QPointF(
          250 + 
            x * climax_ * 
              cos(-new_robot_msg_.initialPose.theta / 180.0 * STDR_PI) 
            - y * climax_ * 
              sin(-new_robot_msg_.initialPose.theta / 180.0 * STDR_PI),
      
          250 + 
            x * climax_ * 
              sin(-new_robot_msg_.initialPose.theta / 180.0 * STDR_PI) 
            + y * climax_ * 
              cos(-new_robot_msg_.initialPose.theta / 180.0 * STDR_PI));
      }
      painter.drawPolyline(points, new_robot_msg_.footprint.points.size() + 1);
      
      painter.setPen(Qt::red);
      
      max_val = sqrt(max_val);
      
      painter.drawLine(  
        250,
        250,
        250 + max_val * climax_ * 1.05 * cos(
          new_robot_msg_.initialPose.theta / 180.0 * STDR_PI),
        250 - max_val * climax_ * 1.05 * sin(
          new_robot_msg_.initialPose.theta / 180.0 * STDR_PI));
    }
    
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
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
      if(laser_hightlight_id_ == i)
      {
        brush = QBrush(QColor(0,200,0,150));
      }
      else
      {
        brush = QBrush(QColor(0,200,0,50));
      }
      painter.setBrush(brush);
      
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
          new_robot_msg_.initialPose.theta + 
          new_robot_msg_.laserSensors[i].pose.theta) * 16,
        
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
      
      if(sonar_hightlight_id_ == i)
      {
        brush = QBrush(QColor(200,0,0,150));
      }
      else
      {
        brush = QBrush(QColor(200,0,0,50));
      }
      painter.setBrush(brush);
      
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
      
      if(rfid_antenna_hightlight_id_ == i)
      {
        brush = QBrush(QColor(0,0,200,30));
      }
      else
      {
        brush = QBrush(QColor(0,0,200,10));
      }
      
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
  @brief Draws the robot's rfid antennas
  **/
  void CRobotCreatorConnector::drawCO2Sensors(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    QBrush brush(QColor(100,0,200,20));
    painter.setBrush(brush);
    float robotOrientation = 
      new_robot_msg_.initialPose.theta / 180.0 * STDR_PI;
    
    for(unsigned int i = 0 ; i < new_robot_msg_.co2Sensors.size() ; i++)
    {
      
      if(co2_sensor_hightlight_id_ == i)
      {
        brush = QBrush(QColor(100,0,200,30));
      }
      else
      {
        brush = QBrush(QColor(100,0,200,10));
      }
      
      float x = new_robot_msg_.co2Sensors[i].pose.x;
      float y = new_robot_msg_.co2Sensors[i].pose.y;
      float newx = 
        x * cos(robotOrientation) - y * sin(robotOrientation);
      float newy = 
        x * sin(robotOrientation) + y * cos(robotOrientation);
      
      painter.drawPie(  
        250 - new_robot_msg_.co2Sensors[i].maxRange * climax_ + newx * climax_,
        250 - new_robot_msg_.co2Sensors[i].maxRange * climax_ - newy * climax_,
        new_robot_msg_.co2Sensors[i].maxRange * climax_ * 2,
        new_robot_msg_.co2Sensors[i].maxRange * climax_ * 2,
        0,
        (2 * 180.0) * 16);
    }
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
  }
  /**
  @brief Draws the robot's rfid antennas
  @return void
  **/
  void CRobotCreatorConnector::drawThermalSensors(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    QBrush brush(QColor(200,0,0,10));
    painter.setBrush(brush);
    float robotOrientation = 
      new_robot_msg_.initialPose.theta / 180.0 * STDR_PI;
    
    for(unsigned int i = 0 ; i < new_robot_msg_.thermalSensors.size() ; i++)
    {
      
      if(thermal_sensor_hightlight_id_ == i)
      {
        brush = QBrush(QColor(200,0,0,30));
      }
      else
      {
        brush = QBrush(QColor(200,0,0,10));
      }
      
      float x = new_robot_msg_.thermalSensors[i].pose.x;
      float y = new_robot_msg_.thermalSensors[i].pose.y;
      float newx = 
        x * cos(robotOrientation) - y * sin(robotOrientation);
      float newy = 
        x * sin(robotOrientation) + y * cos(robotOrientation);
      
      painter.drawPie(  
        250 - new_robot_msg_.thermalSensors[i].maxRange * climax_ + newx * climax_,
        250 - new_robot_msg_.thermalSensors[i].maxRange * climax_ - newy * climax_,
        new_robot_msg_.thermalSensors[i].maxRange * climax_ * 2,
        new_robot_msg_.thermalSensors[i].maxRange * climax_ * 2,
        (new_robot_msg_.thermalSensors[i].pose.theta - 
          new_robot_msg_.thermalSensors[i].angleSpan / 2.0+
          new_robot_msg_.initialPose.theta) * 16,
        (new_robot_msg_.thermalSensors[i].angleSpan) * 16);
    }
    loader_.robotPreviewLabel->setPixmap(
      QPixmap().fromImage(loader_.robotPreviewImage));
  }
  /**
  @brief Draws the robot's rfid antennas
  @return void
  **/
  void CRobotCreatorConnector::drawSoundSensors(void)
  {
    QPainter painter(&loader_.robotPreviewImage);
    QBrush brush(QColor(0,50,200,20));
    painter.setBrush(brush);
    float robotOrientation = 
      new_robot_msg_.initialPose.theta / 180.0 * STDR_PI;
    
    for(unsigned int i = 0 ; i < new_robot_msg_.soundSensors.size() ; i++)
    {
      
      if(sound_sensor_hightlight_id_ == i)
      {
        brush = QBrush(QColor(0,50,200,30));
      }
      else
      {
        brush = QBrush(QColor(0,50,200,10));
      }
      
      float x = new_robot_msg_.soundSensors[i].pose.x;
      float y = new_robot_msg_.soundSensors[i].pose.y;
      float newx = 
        x * cos(robotOrientation) - y * sin(robotOrientation);
      float newy = 
        x * sin(robotOrientation) + y * cos(robotOrientation);
      
      painter.drawPie(  
        250 - new_robot_msg_.soundSensors[i].maxRange * climax_ + newx * climax_,
        250 - new_robot_msg_.soundSensors[i].maxRange * climax_ - newy * climax_,
        new_robot_msg_.soundSensors[i].maxRange * climax_ * 2,
        new_robot_msg_.soundSensors[i].maxRange * climax_ * 2,
        (new_robot_msg_.soundSensors[i].pose.theta - 
          new_robot_msg_.soundSensors[i].angleSpan / 2.0+
          new_robot_msg_.initialPose.theta) * 16,
        (new_robot_msg_.soundSensors[i].angleSpan) * 16);
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
        tr("Resource files (*.yaml *.xml)"));
    
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
        tr("Resource Files (*.yaml *.xml)"));
    
    //!< Not a valid filename
    if (file_name.isEmpty()) { 
      return;
    }

    try {
      new_robot_msg_ = 
        stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>
          (file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      QMessageBox msg;
      msg.setWindowTitle(QString("STDR Parser - Error"));
      msg.setText(QString(ex.what()));
      msg.exec();
      return;
    }

    new_robot_msg_ = stdr_gui_tools::fixRobotAnglesToDegrees(new_robot_msg_);

    CRobotCreatorConnector::laser_number = -1;
    CRobotCreatorConnector::sonar_number = -1;
    CRobotCreatorConnector::rfid_number = -1;
    CRobotCreatorConnector::co2_sensors_number = -1;
    CRobotCreatorConnector::thermal_sensors_number = -1;
    CRobotCreatorConnector::sound_sensors_number = -1;
    
    for(int i = loader_.robotInfoFootprint.childCount() - 1 ; i >=0 ; i--)
    {
      delete loader_.robotInfoFootprint.child(i);
    }
    
    updateRobotTree();

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
