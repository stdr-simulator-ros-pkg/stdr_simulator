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
  
  void spinThreadFunction(void)
  {
    ros::spin();
  }
  
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
    
    icon_move_.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui") + 
      std::string("/resources/images/arrow_move.png")).c_str()), 
      QSize(20,20), QIcon::Normal, QIcon::Off);
        
    icon_delete_.addFile(QString::fromUtf8((
      stdr_gui_tools::getRosPackagePath("stdr_gui")+
      std::string("/resources/images/remove_icon.png")).c_str()), 
      QSize(20,20), QIcon::Normal, QIcon::Off);
  }

  CGuiController::~CGuiController(void)
  {
    
  }
  
  void CGuiController::initializeCommunications(void)
  {
    map_subscriber_ = n_.subscribe(
      "map", 
      1, 
      &CGuiController::receiveMap,
      this);
      
    robot_subscriber_ = n_.subscribe(
      "stdr_server/active_robots", 
      1, 
      &CGuiController::receiveRobots,
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
  
    QObject::connect(
      this,SIGNAL(updateMap()),
      this, SLOT(updateMapInternal()));
    
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
  }
  
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
    int originx = msg.info.origin.position.x;
    int originy = msg.info.origin.position.y;
    painter.setPen(Qt::blue);
    painter.drawLine(originx, originy - 20, originx, originy + 20);
    painter.drawLine(originx - 20, originy, originx + 20, originy);
    
    initial_map_ = running_map_;

    gui_connector_.setMapLoaded(true);
    info_connector_.updateMapInfo( msg.info.width * msg.info.resolution,
                  msg.info.height * msg.info.resolution,
                  msg.info.resolution);
    map_connector_.setInitialImageSize(
      QSize(initial_map_.width(),initial_map_.height()));
    
    elapsed_time_.start();
    
    timer_->start(50);
  }
  
  void CGuiController::saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg,
    QString file_name)
  {
    std::string file_name_str=file_name.toStdString();
    
    stdr_robot::parser::robotMsgToYaml(file_name_str,newRobotMsg);
    
  }
  
  void CGuiController::loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg)
  {
    Q_EMIT waitForRobotPose();
  }
  
  void CGuiController::loadRfidPressed(void)
  {
    Q_EMIT waitForRfidPose();
  }
  
  void CGuiController::loadThermalPressed(void)
  {
    Q_EMIT waitForThermalPose();
  }
  
  void CGuiController::loadCo2Pressed(void)
  {
    Q_EMIT waitForCo2Pose();
  }
  
  void CGuiController::zoomInPressed(QPoint p)
  {
    map_connector_.updateZoom(p,true);
  }

  void CGuiController::zoomOutPressed(QPoint p)
  {
    map_connector_.updateZoom(p,false);
  }

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
  
  void CGuiController::receiveRobots(
    const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
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
      //~ ROS_ERROR("Place got : %f %f",
        //~ msg.robots[i].robot.initialPose.x,
        //~ msg.robots[i].robot.initialPose.y);
    }
    info_connector_.updateTree(msg);
    map_lock_ = false;
  }
  
  void CGuiController::robotPlaceSet(QPoint p)
  {
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
    gui_connector_.robotCreatorConn.fixRobotMsgAngles();
    try 
    {
      newRobot = robot_handler_.spawnNewRobot(
        gui_connector_.robotCreatorConn.getNewRobot());
        
    }
    catch (ConnectionException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    my_robots_.insert(newRobot.name);
    map_lock_ = false;
  }
  
  void CGuiController::rfidPlaceSet(QPoint p)
  {
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
  
  void CGuiController::co2PlaceSet(QPoint p)
  {
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
  
  void CGuiController::thermalPlaceSet(QPoint p)
  {
    while(map_lock_)
    {	
      usleep(100);
    }
    map_lock_ = true;
    QPoint pnew = map_connector_.getGlobalPoint(p);
    QString name=QString("thermal_source_") + QString().setNum(thermal_sources_.size());
    CGuiThermalSource new_source(pnew,name.toStdString());
    thermal_sources_.insert(std::pair<QString,CGuiThermalSource>(name,new_source));
    map_lock_ = false;
  }
  
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

    for(RfidTagIterator it = rfid_tags_.begin() ; it != rfid_tags_.end() ; it++)
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
      //~ ROS_ERROR("Exit triggered to controller");
      QEvent *e = gui_connector_.getCloseEvent();
      
      this->exit();
      gui_connector_.shutdown();
    }
  }
  
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
  
  void CGuiController::itemClicked(QPoint p,Qt::MouseButton b)
  {
    QPoint pointClicked = map_connector_.getGlobalPoint(p);
    for(unsigned int i = 0 ; i < registered_robots_.size() ; i++)
    {
      if(registered_robots_[i].checkEventProximity(pointClicked))
      {
        if(b == Qt::RightButton)
        {
          QMenu myMenu;
          
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
  
  void CGuiController::robotReplaceSet(QPoint p,std::string robotName)
  {
    QPoint pnew = map_connector_.getGlobalPoint(p);
    
    geometry_msgs::Pose2D newPose;
    newPose.x = pnew.x() * map_msg_.info.resolution;
    newPose.y = pnew.y() * map_msg_.info.resolution;
    
    robot_handler_.moveRobot(robotName,newPose);
  }
}


