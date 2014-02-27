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


namespace stdr_gui
{
  /**
  @brief Thread that performs the ros::spin functionality
  @return void
  **/
  void spinThreadFunction(void)
  {
    ros::spin();
  }
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CGuiController::CGuiController(int argc,char **argv):
    gui_connector_(argc,argv),
    info_connector_(argc,argv),
    map_connector_(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    setupWidgets();
  
    robot_following_ = "";
  
    map_lock_ = false;
    map_initialized_ = false;
    
    icon_move_.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
      std::string("/resources/images/arrow_move.png")).c_str()), 
      QSize(20,20), QIcon::Normal, QIcon::Off);
        
    icon_delete_.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui")+
      std::string("/resources/images/remove_icon.png")).c_str()), 
      QSize(20,20), QIcon::Normal, QIcon::Off);
  }

  /**
  @brief Default destructor
  @return void
  **/
  CGuiController::~CGuiController(void)
  {
    
  }
  
  /**
  @brief Initializes the Qt event connections and ROS subscribers and publishers
  @return void
  **/
  void CGuiController::initializeCommunications(void)
  {
    map_subscriber_ = n_.subscribe(
      "map", 
      1, 
      &CGuiController::receiveMap,
      this);
    
    QObject::connect(
      &gui_connector_,SIGNAL(setZoomInCursor(bool)),
      &map_connector_, SLOT(setCursorZoomIn(bool)));
    
    QObject::connect(
      &gui_connector_,SIGNAL(setZoomOutCursor(bool)),
      &map_connector_, SLOT(setCursorZoomOut(bool)));
    
    QObject::connect(
      &gui_connector_,SIGNAL(setAdjustedCursor(bool)),
      &map_connector_, SLOT(setCursorAdjusted(bool)));
    
    QObject::connect(
      &map_connector_,SIGNAL(zoomInPressed(QPoint)),
      this, SLOT(zoomInPressed(QPoint)));
    
    QObject::connect(
      &map_connector_,SIGNAL(zoomOutPressed(QPoint)),
      this, SLOT(zoomOutPressed(QPoint)));
    
    QObject::connect(
      &map_connector_,SIGNAL(itemClicked(QPoint,Qt::MouseButton)),
      this, SLOT(itemClicked(QPoint,Qt::MouseButton)));
    
    QObject::connect(
      &info_connector_,SIGNAL(laserVisualizerClicked(QString,QString)),
      this, SLOT(laserVisualizerClicked(QString,QString)));
    
    QObject::connect(
      &info_connector_,SIGNAL(sonarVisualizerClicked(QString,QString)),
      this, SLOT(sonarVisualizerClicked(QString,QString)));
      
    QObject::connect(
      &info_connector_,SIGNAL(robotVisualizerClicked(QString)),
      this, SLOT(robotVisualizerClicked(QString)));
    
    QObject::connect(
      &(gui_connector_.robotCreatorConn),
        SIGNAL(saveRobotPressed(stdr_msgs::RobotMsg,QString)),
      this, SLOT(saveRobotPressed(stdr_msgs::RobotMsg,QString)));
    
    QObject::connect(
      &(gui_connector_.robotCreatorConn),
        SIGNAL(loadRobotPressed(stdr_msgs::RobotMsg)),
      this, SLOT(loadRobotPressed(stdr_msgs::RobotMsg)));
      
    QObject::connect(
      &(gui_connector_),
        SIGNAL(robotFromFile(stdr_msgs::RobotMsg)),
      this, SLOT(loadRobotFromFilePressed(stdr_msgs::RobotMsg)));  
      
    QObject::connect(
      &(gui_connector_),
        SIGNAL(loadRfidPressed()),
      this, SLOT(loadRfidPressed()));
    
    QObject::connect(
      &(gui_connector_),
        SIGNAL(loadThermalPressed()),
      this, SLOT(loadThermalPressed()));
    
    QObject::connect(
      &(gui_connector_),
        SIGNAL(loadCo2Pressed()),
      this, SLOT(loadCo2Pressed()));
    
    QObject::connect(
      this,SIGNAL(waitForRobotPose()),
      &map_connector_, SLOT(waitForPlace()));
    
    QObject::connect(
      &map_connector_,SIGNAL(robotPlaceSet(QPoint)),
      this, SLOT(robotPlaceSet(QPoint)));
      
    QObject::connect(
      this,SIGNAL(replaceRobot(std::string)),
      &map_connector_, SLOT(waitForReplace(std::string)));
      
    QObject::connect(
      &map_connector_,SIGNAL(robotReplaceSet(QPoint,std::string)),
      this, SLOT(robotReplaceSet(QPoint,std::string)));
    
    timer_ = new QTimer(this);
    connect(
      timer_, SIGNAL(timeout()), 
      this, SLOT(updateMapInternal()));
      
    QObject::connect(
      this,SIGNAL(waitForRfidPose()),
      &map_connector_, SLOT(waitForRfidPlace()));
    
    QObject::connect(
      &map_connector_,SIGNAL(rfidPlaceSet(QPoint)),
      this, SLOT(rfidPlaceSet(QPoint)));
      
    QObject::connect(
      this,SIGNAL(waitForThermalPose()),
      &map_connector_, SLOT(waitForThermalPlace()));
    
    QObject::connect(
      &map_connector_,SIGNAL(thermalPlaceSet(QPoint)),
      this, SLOT(thermalPlaceSet(QPoint)));
      
    QObject::connect(
      this,SIGNAL(waitForCo2Pose()),
      &map_connector_, SLOT(waitForCo2Place()));
    
    QObject::connect(
      &map_connector_,SIGNAL(co2PlaceSet(QPoint)),
      this, SLOT(co2PlaceSet(QPoint)));
      
    QObject::connect(
      &info_connector_,SIGNAL(laserVisibilityClicked(QString,QString)),
      this, SLOT(laserVisibilityClicked(QString,QString)));
      
    QObject::connect(
      &info_connector_,SIGNAL(sonarVisibilityClicked(QString,QString)),
      this, SLOT(sonarVisibilityClicked(QString,QString)));
      
    QObject::connect(
      &info_connector_,SIGNAL(robotVisibilityClicked(QString)),
      this, SLOT(robotVisibilityClicked(QString)));
    
    QObject::connect(
      this, SIGNAL(setRobotVisibility(QString,char)),
      &info_connector_, SLOT(setRobotVisibility(QString,char)));
    
    QObject::connect(
      this, SIGNAL(setLaserVisibility(QString,QString,char)),
      &info_connector_, SLOT(setLaserVisibility(QString,QString,char)));
    
    QObject::connect(
      this, SIGNAL(setSonarVisibility(QString,QString,char)),
      &info_connector_, SLOT(setSonarVisibility(QString,QString,char)));
  }
  
  /**
  @brief Sets up the main window widgets
  @return void
  **/
  void CGuiController::setupWidgets(void)
  {
    {
      gui_connector_.addToGrid(info_connector_.getLoader(),0,0);
    }
    {
      initial_map_ = running_map_ = QImage((
        stdr_gui_tools::getRosPackagePath("stdr_gui") + 
        std::string("/resources/images/logo.png")).c_str());

      map_msg_.info.width = initial_map_.width();
      map_msg_.info.height = initial_map_.height();
      
      map_connector_.updateImage(&running_map_);
      
      gui_connector_.addToGrid(map_connector_.getLoader(),0,1);

      gui_connector_.setGridColumnStretch(1,5);
      gui_connector_.setGridColumnStretch(0,2);
    }
  }
  
  /**
  @brief Initializes the ROS spin and Qt threads
  @return bool
  **/
  bool CGuiController::init(void)
  {
    if ( ! ros::master::check() ) 
    {
      return false;
    }
    gui_connector_.show();

    initializeCommunications();
    boost::thread spinThread(&spinThreadFunction);
    return true;
  }

  /**
  @brief Receives the occupancy grid map from stdr_server. Connects to "map" ROS topic
  @param msg [const nav_msgs::OccupancyGrid&] The OGM message
  @return void
  **/
  void CGuiController::receiveMap(const nav_msgs::OccupancyGrid& msg)
  {
    map_msg_ = msg;
    initial_map_ = running_map_ =
      QImage(msg.info.width,msg.info.height,QImage::Format_RGB32);
    QPainter painter(&running_map_);
    int d(0);
    QColor c;
    for( unsigned int i = 0 ; i < msg.info.width ; i++ )
    {
      for( unsigned int j = 0 ; j < msg.info.height ; j++ )
      {
        if( msg.data[j * msg.info.width + i] == -1 )
        {
          c=QColor(127,127,127);
        }
        else
        {
          d = (100.0 - msg.data[j * msg.info.width + i]) / 100.0 * 255.0;
          c=QColor(d,d,d);
        }
        painter.setPen(c);
        painter.drawPoint(i,j);
      }
    }
    int originx = msg.info.origin.position.x / msg.info.resolution;
    int originy = msg.info.origin.position.y / msg.info.resolution;
    painter.setPen(Qt::blue);
    painter.drawLine(originx, originy - 20, originx, originy + 20);
    painter.drawLine(originx - 20, originy, originx + 20, originy);
    
    initial_map_ = running_map_;

    info_connector_.updateMapInfo( msg.info.width * msg.info.resolution,
                  msg.info.height * msg.info.resolution,
                  msg.info.resolution);
    map_connector_.setInitialImageSize(
      QSize(initial_map_.width(),initial_map_.height()));
    
    elapsed_time_.start();
    
    map_initialized_ = true;
    map_connector_.setMapInitialized(true);
    gui_connector_.setMapInitialized(true);
    
    timer_->start(50);
    
    robot_subscriber_ = n_.subscribe(
      "stdr_server/active_robots", 
      1, 
      &CGuiController::receiveRobots,
      this);
  }
  
  /**
  @brief Saves the robot in a file. Connects to the CGuiConnector::CRobotCreatorConnector::saveRobotPressed signal
  @param newRobotMsg [stdr_msgs::RobotMsg] The robot to be saved
  @param file_name [QString] The file for the robot to be saved
  @return void
  **/
  void CGuiController::saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg,
    QString file_name)
  {
    std::string file_name_str=file_name.toStdString();
    
    try {
      stdr_parser::Parser::saveMessage(newRobotMsg, file_name.toStdString());
    }
    catch(stdr_parser::ParserException ex)
    {
      gui_connector_.raiseMessage("STDR Parser - Error", ex.what());
    }
  }
  
  /**
  @brief Loads a robot from robot creator into map. Connects to the CGuiConnector::CRobotCreatorConnector::loadRobotPressed signal
  @param newRobotMsg [stdr_msgs::RobotMsg] The robot to be put in the map
  @return void
  **/
  void CGuiController::loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT waitForRobotPose();
  }
  
  /**
  @brief Informs CGuiController that a robot is loaded from a yaml file. Connects to the CGuiConnector::robotFromFile signal
  @param newRobotMsg [stdr_msgs::RobotMsg] The robot to be put in the map
  @return void
  **/
  void CGuiController::loadRobotFromFilePressed(stdr_msgs::RobotMsg newRobotMsg)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    gui_connector_.robotCreatorConn.setNewRobot(
      stdr_gui_tools::fixRobotAnglesToDegrees(newRobotMsg));
    Q_EMIT waitForRobotPose();
  }
  
  /**
  @brief Informs CGuiController that an RFID is going to be placed in the environment. Connects to the CGuiConnector::loadRfidPressed signal
  @return void
  **/
  void CGuiController::loadRfidPressed(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT waitForRfidPose();
  }
  
  /**
  @brief Informs CGuiController that a thermal source is going to be placed in the environment. Connects to the CGuiConnector::loadThermalPressed signal
  @return void
  **/
  void CGuiController::loadThermalPressed(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT waitForThermalPose();
  }
  
  /**
  @brief Informs CGuiController that a CO2 source is going to be placed in the environment. Connects to the CGuiConnector::loadCo2Pressed signal
  @return void
  **/
  void CGuiController::loadCo2Pressed(void)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    Q_EMIT waitForCo2Pose();
  }
  
  /**
  @brief Performs zoom in when the button is pressed. Connects to the CMapConnector::zoomInPressed signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::zoomInPressed(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    map_connector_.updateZoom(p,true);
  }

  /**
  @brief Performs zoom out when the button is pressed. Connects to the CMapConnector::zoomOutPressed signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::zoomOutPressed(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    map_connector_.updateZoom(p,false);
  }

  /**
  @brief Dumps all visualizers that connect to robots not existent in the input argument message
  @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The robots message
  @return void
  **/
  void CGuiController::cleanupVisualizers(
    const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
    std::set<QString> newSensors, erasedSensorsL, erasedSensorsS;
    for( unsigned int r = 0 ; r < msg.robots.size() ; r++ )
    {
      QString baseName = QString(msg.robots[r].name.c_str());
      for(unsigned int l = 0; 
        l < msg.robots[r].robot.laserSensors.size() ; l++)
      {
        QString fullName = baseName + QString("/") + 
          QString(
            msg.robots[r].robot.laserSensors[l].frame_id.c_str() );
        newSensors.insert(fullName);
      }
      for(unsigned int s = 0;
        s < msg.robots[r].robot.sonarSensors.size() ; s++ )
      {
        QString fullName = baseName + QString("/") + 
          QString(
            msg.robots[r].robot.sonarSensors[s].frame_id.c_str());
        newSensors.insert(fullName);
      }
    }
    
    for(LaserVisIterator it = laser_visualizers_.begin() ; 
      it != laser_visualizers_.end() ; it++ )
    {
      if( newSensors.find(it->first) == newSensors.end() )
      {
        erasedSensorsL.insert(it->first);
      }
    }
    for(SonarVisIterator it = sonar_visualizers_.begin() ; 
      it != sonar_visualizers_.end() ; it++)
    {
      if(newSensors.find(it->first) == newSensors.end())
      {
        erasedSensorsS.insert(it->first);
      }
    }
    
    for(std::set<QString>::iterator it = erasedSensorsL.begin() ; 
      it != erasedSensorsL.end() ; it++)
    {
      laser_visualizers_[*it]->destruct();
      laser_visualizers_.erase(*it);
    }
    for(std::set<QString>::iterator it = erasedSensorsS.begin() ; 
      it != erasedSensorsS.end() ; it++)
    {
      sonar_visualizers_[*it]->destruct();
      sonar_visualizers_.erase(*it);
    }
      
  }
  
  /**
  @brief Receives the robots from stdr_server. Connects to "stdr_server/active_robots" ROS topic
  @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The robots message
  @return void
  **/
  void CGuiController::receiveRobots(
    const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    while(map_lock_)	
    {
      usleep(100);
    }
    map_lock_ = true;
    
    cleanupVisualizers(msg);
    
    registered_robots_.clear();
    all_robots_ = msg;
    for(unsigned int i = 0 ; i < msg.robots.size() ; i++)
    {
      registered_robots_.push_back(CGuiRobot(msg.robots[i]));
    }
    info_connector_.updateTree(msg);
    map_lock_ = false;
  }
  
  /**
  @brief Gets the point at which the new robot is placed. Connects to the CMapConnector::robotPlaceSet signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::robotPlaceSet(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    while(map_lock_)
    { 
      usleep(100);
    }
    map_lock_ = true;
    
    QPoint pnew = map_connector_.getGlobalPoint(p);
    gui_connector_.robotCreatorConn.setInitialPose(
      pnew.x() * map_msg_.info.resolution,
      pnew.y() * map_msg_.info.resolution);
      
    stdr_msgs::RobotIndexedMsg newRobot;
    try 
    {
      newRobot = robot_handler_.spawnNewRobot(
        stdr_gui_tools::fixRobotAnglesToRad(
          gui_connector_.robotCreatorConn.getNewRobot()));
        
    }
    catch (stdr_robot::ConnectionException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    my_robots_.insert(newRobot.name);
    map_lock_ = false;
  }
  
  /**
  @brief Gets the point at which the new RFID tag is placed. Connects to the CMapConnector::robotPlaceSet signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::rfidPlaceSet(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    while(map_lock_)
    {	
      usleep(100);
    }
    map_lock_ = true;
    
    QPoint pnew = map_connector_.getGlobalPoint(p);
    QString name=QString("rfid_tag_") + QString().setNum(rfid_tags_.size());
    CGuiRfidTag new_tag(pnew,name.toStdString());
    
    bool ok;
            
    //~ QString message = QInputDialog::getText(
      //~ this, tr("QInputDialog::getText()"),
      //~ tr("User name:"), QLineEdit::Normal,
      //~ QDir::home().dirName(), &ok);
    //~ if ( ok && !message.isEmpty() ) {
        //~ new_tag.setMessage(message);
    //~ }

    rfid_tags_.insert(std::pair<QString,CGuiRfidTag>(name,new_tag));
    map_lock_ = false;
  }
  
  /**
  @brief Gets the point at which the new CO2 source is placed. Connects to the CMapConnector::co2PlaceSet signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::co2PlaceSet(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    while(map_lock_)
    {	
      usleep(100);
    }
    map_lock_ = true;
    QPoint pnew = map_connector_.getGlobalPoint(p);
    QString name=QString("co2_source_") + QString().setNum(co2_sources_.size());
    CGuiCo2Source new_source(pnew,name.toStdString());
    co2_sources_.insert(std::pair<QString,CGuiCo2Source>(name,new_source));
    map_lock_ = false;
  }
  
  /**
  @brief Gets the point at which the new thermal source is placed. Connects to the CMapConnector::thermalPlaceSet signal
  @param p [QPoint] The event point in the OGM
  @return void
  **/
  void CGuiController::thermalPlaceSet(QPoint p)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    while(map_lock_)
    {	
      usleep(100);
    }
    map_lock_ = true;
    QPoint pnew = map_connector_.getGlobalPoint(p);
    QString name=QString("thermal_source_") + 
      QString().setNum(thermal_sources_.size());
    CGuiThermalSource new_source(pnew,name.toStdString());
    thermal_sources_.insert(
      std::pair<QString,CGuiThermalSource>(name,new_source));
    map_lock_ = false;
  }
  
  /**
  @brief Updates the map to be shown in GUI. Connects to the timeout signal of timer_
  @return void
  **/
  void CGuiController::updateMapInternal(void)
  {
    while(map_lock_)
    {
      usleep(100);
    }
    map_lock_ = true;
    running_map_ = initial_map_;
    
    if(gui_connector_.isGridEnabled())
      map_connector_.drawGrid(&running_map_, map_msg_.info.resolution);
    
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      registered_robots_[i].draw(
        &running_map_,map_msg_.info.resolution,&listener_);
    }
    running_map_ = running_map_.mirrored(false,true);
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].getShowLabel())
        registered_robots_[i].drawLabel(
          &running_map_,map_msg_.info.resolution);
    }
    
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].getFrameId() == robot_following_)
      {
        map_connector_.updateCenter(
          registered_robots_[i].getCurrentPose());
      }
    }

    for(RfidTagIterator it = rfid_tags_.begin() ; 
      it != rfid_tags_.end() ; it++)
    {
      it->second.draw(&running_map_);
    }
    
    for(Co2SourcesIterator it = co2_sources_.begin() ; 
      it != co2_sources_.end() ; it++)
    {
      it->second.draw(&running_map_);
    }
    
    for(ThermalSourcesIterator it = thermal_sources_.begin() ; 
      it != thermal_sources_.end() ; it++)
    {
      it->second.draw(&running_map_);
    }
    
    map_connector_.updateImage(&running_map_);
    
    gui_connector_.setStatusBarMessage(
      QString("Time elapsed : ") + 
      stdr_gui_tools::getLiteralTime(elapsed_time_.elapsed()));
    map_lock_ = false;
  
    //!<--------------------------- Check if all visualisers are active
    std::vector<QString> toBeErased;
    for(LaserVisIterator it = laser_visualizers_.begin() ; 
      it != laser_visualizers_.end() ; it++)
    {
      if( ! it->second->getActive())
      {
        toBeErased.push_back(it->first);
      }
      else
      {
        it->second->paint();
      }
    }
    for(unsigned int i = 0 ; i < toBeErased.size() ; i++)
    {
      laser_visualizers_.erase(toBeErased[i]);
    }
    toBeErased.clear();
    for(SonarVisIterator it = sonar_visualizers_.begin() ; 
      it != sonar_visualizers_.end() ; it++)
    {
      if( ! it->second->getActive())
      {
        toBeErased.push_back(it->first);
      }
      else
      {
        it->second->paint();
      }
    }
    for(unsigned int i = 0 ; i < toBeErased.size() ; i++)
    {
      sonar_visualizers_.erase(toBeErased[i]);
    }
    toBeErased.clear();
    for(RobotVisIterator it = robot_visualizers_.begin() ; 
      it != robot_visualizers_.end() ; 
      it++)
    {
      if( ! it->second->getActive())
      {
        toBeErased.push_back(it->first);
      }
      else
      {
        QString robotName = it->first;
        for(unsigned int r = 0 ; r < registered_robots_.size() ; r++)
        {
          if(registered_robots_[r].getFrameId() == robotName.toStdString())
          {
            it->second->setImage(
              registered_robots_[r].
                getVisualization(map_msg_.info.resolution));
            it->second->setCurrentPose(registered_robots_[r].getCurrentPoseM());
            
            it->second->setCurrentSpeed(
              registered_robots_[r].getSpeeds());
            break;
          }
        }
      }
    }
    for(unsigned int i = 0 ; i < toBeErased.size() ; i++)
    {
      robot_visualizers_.erase(toBeErased[i]);
    }
    
    //!< -----------------------------------------Check for close event
    if(gui_connector_.closeTriggered())
    {
      QEvent *e = gui_connector_.getCloseEvent();
      
      this->exit();
      gui_connector_.shutdown();
    }
  }
  
  /**
  @brief Returns a stdr_msgs::LaserSensorMsg message from robot and laser name
  @param robotName [QString] Frame id of the robot
  @param laserName [QString] Frame id of the laser
  @return stdr_msgs::LaserSensorMsg
  **/
  stdr_msgs::LaserSensorMsg CGuiController::getLaserDescription(
    QString robotName,
    QString laserName)
  {
    for(unsigned int i = 0 ; i < all_robots_.robots.size() ; i++)
    {
      if(all_robots_.robots[i].name == robotName.toStdString())
      {
        for(unsigned int j = 0 ; 
          j < all_robots_.robots[i].robot.laserSensors.size() ; 
          j++)
        {
          if(all_robots_.robots[i].robot.laserSensors[j].frame_id
               == laserName.toStdString())
          {
            return all_robots_.robots[i].robot.laserSensors[j];
          }
        }
      }
    }
    return stdr_msgs::LaserSensorMsg();
  }
  
  /**
  @brief Returns a stdr_msgs::SonarSensorMsg message from robot and sonar name
  @param robotName [QString] Frame id of the robot
  @param sonarName [QString] Frame id of the sonar
  @return stdr_msgs::SonarSensorMsg
  **/
  stdr_msgs::SonarSensorMsg CGuiController::getSonarDescription(
    QString robotName,
    QString sonarName)
  {
    for(unsigned int i = 0 ; i < all_robots_.robots.size() ; i++)
    {
      if(all_robots_.robots[i].name == robotName.toStdString())
      {
        for(unsigned int j = 0 ; 
          j < all_robots_.robots[i].robot.sonarSensors.size() ; 
          j++)
        {
          if(all_robots_.robots[i].robot.sonarSensors[j].frame_id
               == sonarName.toStdString())
          {
            return all_robots_.robots[i].robot.sonarSensors[j];
          }
        }
      }
    }	
    return stdr_msgs::SonarSensorMsg();
  }
  
  /**
  @brief Informs CGuiController that a laser visualizer has been clicked. Connects to the CInfoConnector::laserVisualizerClicked signal
  @param robotName [QString] Frame id of the robot
  @param laserName [QString] Frame id of the laser
  @return void
  **/
  void CGuiController::laserVisualizerClicked(
    QString robotName,
    QString laserName)
  {
    QString name = robotName + QString("/") + laserName;
    if(laser_visualizers_.find(name) != laser_visualizers_.end())
    {
      return;
    }
    CLaserVisualisation *lv;
    lv = new CLaserVisualisation(name,map_msg_.info.resolution);
    laser_visualizers_.insert(
      std::pair<QString,CLaserVisualisation *>(name,lv));
    lv->setWindowFlags(Qt::WindowStaysOnTopHint);

    lv->setLaser(getLaserDescription(robotName,laserName));
    
    lv->show();
  }
  
  /**
  @brief Informs CGuiController that a sonar visualizer has been clicked. Connects to the CInfoConnector::sonarVisualizerClicked signal
  @param robotName [QString] Frame id of the robot
  @param sonarName [QString] Frame id of the sonar
  @return void
  **/
  void CGuiController::sonarVisualizerClicked(
    QString robotName,
    QString sonarName)
  {
    QString name = robotName + QString("/") + sonarName;
    if(sonar_visualizers_.find(name) != sonar_visualizers_.end())
    {
      return;
    }
    CSonarVisualisation *sv;
    sv = new CSonarVisualisation(name,map_msg_.info.resolution);
    sonar_visualizers_.insert(
      std::pair<QString,CSonarVisualisation *>(name,sv));
    sv->setWindowFlags(Qt::WindowStaysOnTopHint);

    sv->setSonar(getSonarDescription(robotName,sonarName));
            
    sv->show();
  }
  
  /**
  @brief Informs CGuiController that a robot visualizer has been clicked. Connects to the CInfoConnector::robotVisualizerClicked signal
  @param robotName [QString] Frame id of the robot
  @return void
  **/
  void CGuiController::robotVisualizerClicked(QString robotName)
  {
    QString name = robotName;
    if(robot_visualizers_.find(name) != robot_visualizers_.end())
    {
      return;
    }
    CRobotVisualisation *sv;
    sv = new CRobotVisualisation(name,map_msg_.info.resolution);
    robot_visualizers_.insert(
      std::pair<QString,CRobotVisualisation *>(name,sv));
    sv->setWindowFlags(Qt::WindowStaysOnTopHint);

    sv->show();
  }
  
  /**
  @brief Informs CGuiController that click has performed in the map. Connects to the CMapConnector::itemClicked signal
  @param p [QPoint] The event point in map
  @param b [Qt::MouseButton] The mouse button used to trigger the event
  @return void
  **/
  void CGuiController::itemClicked(QPoint p,Qt::MouseButton b)
  {
    gui_connector_.uncheckZoomButtons();
    QPoint pointClicked = map_connector_.getGlobalPoint(p);
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].checkEventProximity(pointClicked))
      {
        if(b == Qt::RightButton)
        {
          QMenu myMenu;
          
          QAction *nothing = myMenu.addAction(
            QString("Robot : ") + 
            QString(registered_robots_[i].getFrameId().c_str()));
          nothing->setCheckable(false);
          nothing->setEnabled(false);
          
          QAction *deleteRobot = myMenu.addAction(icon_delete_,"Delete robot");
          QAction *moveRobot = myMenu.addAction(icon_move_,"Move robot");
          myMenu.addSeparator();
          QAction *showCircle = myMenu.addAction("Show proximity circles");
          QAction *followRobot = myMenu.addAction("Follow robot");
          
          QAction* selectedItem = myMenu.exec(map_connector_.mapToGlobal(p));
          if(selectedItem == showCircle)
          {
            registered_robots_[i].toggleShowCircles();
          }
          else if(selectedItem == deleteRobot)
          {
            robot_handler_.deleteRobot(
              registered_robots_[i].getFrameId());
          }
          else if(selectedItem == moveRobot)
          {
            Q_EMIT replaceRobot(registered_robots_[i].getFrameId());
          }
          else if(selectedItem == followRobot)
          {
            if(robot_following_ == registered_robots_[i].getFrameId())
            {
              robot_following_ = "";
            }
            else
            {
              robot_following_ = registered_robots_[i].getFrameId();
            }
          }
        }
        else if(b == Qt::LeftButton)
        {
          registered_robots_[i].toggleShowLabel();
        }
      }
    }
  }
  
  /**
  @brief Informs CGuiController about the new pose of a robot. Connects to the CMapConnector::robotReplaceSet signal
  @param p [QPoint] The event point in map
  @param robotName [std::string] The frame id of the re-placed robot
  @return void
  **/
  void CGuiController::robotReplaceSet(QPoint p,std::string robotName)
  {
    if ( ! map_initialized_ )
    {
      return;
    }
    QPoint pnew = map_connector_.getGlobalPoint(p);
    
    geometry_msgs::Pose2D newPose;
    newPose.x = pnew.x() * map_msg_.info.resolution;
    newPose.y = pnew.y() * map_msg_.info.resolution;
    
    bool success = robot_handler_.moveRobot(robotName,newPose);
    if(!success)
    {
      gui_connector_.raiseMessage(
        "STDR robot - Error", "Unable to relocate the robot");
    }
  }
  
  /**
  @brief Informs CGuiController that a laser visibility status has been clicked. Connects to the CInfoConnector::laserVisibilityClicked signal
  @param robotName [QString] Frame id of the robot
  @param laserName [QString] Frame id of the laser
  @return void
  **/
  void CGuiController::laserVisibilityClicked(
    QString robotName,QString laserName)
  {
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].getFrameId() == robotName.toStdString())
      {
        char vs = registered_robots_[i].getLaserVisualizationStatus(
          laserName.toStdString());
        Q_EMIT setLaserVisibility(robotName,laserName,(vs + 1) % 3);
        registered_robots_[i].toggleLaserVisualizationStatus(
          laserName.toStdString());
        break;
      }
    }
  }
  
  /**
  @brief Informs CGuiController that a sonar visibility status has been clicked. Connects to the CInfoConnector::sonarVisibilityClicked signal
  @param robotName [QString] Frame id of the robot
  @param sonarName [QString] Frame id of the sonar
  @return void
  **/
  void CGuiController::sonarVisibilityClicked(
    QString robotName,QString sonarName)
  {
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].getFrameId() == robotName.toStdString())
      {
        char vs = registered_robots_[i].getSonarVisualizationStatus(
          sonarName.toStdString());
        Q_EMIT setSonarVisibility(robotName,sonarName,(vs + 1) % 3);
        registered_robots_[i].toggleSonarVisualizationStatus(
          sonarName.toStdString());
        break;
      }
    }
  }
  
  /**
  @brief Informs CGuiController that a robot visibility status has been clicked. Connects to the CInfoConnector::robotVisibilityClicked signal
  @param robotName [QString] Frame id of the robot
  @return void
  **/
  void CGuiController::robotVisibilityClicked(QString robotName)
  {
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].getFrameId() == robotName.toStdString())
      {
        char vs = registered_robots_[i].getVisualizationStatus();
        Q_EMIT setRobotVisibility(robotName,(vs + 1) % 3);
        registered_robots_[i].toggleVisualizationStatus();
      }
    }
  }
}


