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

#ifndef STDR_GUI_CONTROLLER
#define STDR_GUI_CONTROLLER

#include <iostream>
#include <cstdlib>
#include <boost/thread.hpp>

#include "nav_msgs/OccupancyGrid.h"

#include "stdr_gui/stdr_gui_connector.h"
#include "stdr_gui/stdr_info_connector.h"
#include "stdr_gui/stdr_map_connector.h"
#include "stdr_gui/stdr_visualization/stdr_sonar_visualization.h"
#include "stdr_gui/stdr_visualization/stdr_laser_visualization.h"
#include "stdr_gui/stdr_visualization/stdr_robot_visualization.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_co2_source.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_thermal_source.h"
#include "stdr_gui/stdr_map_metainformation/stdr_gui_rfid_tag.h"

#include <stdr_robot/handle_robot.h>

namespace stdr_gui
{

  /**
  @class GuiController
  @brief The main controller for the STDR GUI. Inherits QThread
  **/ 
  class CGuiController : 
    public QThread
  {
    Q_OBJECT
    
    private: 
    
      typedef std::map<QString,CLaserVisualisation *>::iterator
        LaserVisIterator;
      typedef std::map<QString,CSonarVisualisation *>::iterator
        SonarVisIterator;
      typedef std::map<QString,CRobotVisualisation *>::iterator
        RobotVisIterator;

      int  argc_;
      char** argv_;
      
      bool 	map_lock_;
      
      std::vector<CGuiRobot> registered_robots_;
      std::set<std::string> my_robots_;
      
      std::map<QString,CLaserVisualisation *> laser_visualizers_;
      std::map<QString,CSonarVisualisation *> sonar_visualizers_;
      std::map<QString,CRobotVisualisation *> robot_visualizers_;

      std::map<QString,CGuiRfidTag> rfid_tags_;
      std::map<QString,CGuiThermalSource> thermal_source_;
      std::map<QString,CGuiCo2Source> co2_source_;
      
      ros::Subscriber map_subscriber_;
      ros::Subscriber robot_subscriber_;
      ros::NodeHandle n_;
      tf::TransformListener listener_;
      
      nav_msgs::OccupancyGrid map_msg_;

      stdr_robot::HandleRobot robot_handler_;
      stdr_msgs::RobotIndexedVectorMsg all_robots_;

      QTimer* timer_;
      QTime elapsed_time_;
      QIcon icon_move_;
      QIcon icon_delete_;
      QImage initial_map_;
      QImage running_map_;

      CGuiConnector gui_connector_;
      CInfoConnector info_connector_;
      CMapConnector map_connector_;

      stdr_msgs::LaserSensorMsg getLaserDescription(
        QString robotName,
        QString laserName); 

      stdr_msgs::SonarSensorMsg getSonarDescription(
        QString robotName,
        QString sonarName); 
        
      std::string robot_following_;
      
    public:
      CGuiController(int argc,char **argv);
      ~CGuiController(void);
      
      void setupWidgets(void);
      void initializeCommunications(void);
      void receiveMap(const nav_msgs::OccupancyGrid& msg);
      void receiveRobots(const stdr_msgs::RobotIndexedVectorMsg& msg);
      bool init();
      void cleanupVisualizers(const stdr_msgs::RobotIndexedVectorMsg& msg);
    
    public Q_SLOTS:
      void saveRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
      void loadRobotPressed(stdr_msgs::RobotMsg newRobotMsg);
      void loadRfidPressed(void);
      void loadCo2Pressed(void);
      void loadThermalPressed(void);
      void zoomInPressed(QPoint p);
      void zoomOutPressed(QPoint p);
      void robotPlaceSet(QPoint p);
      void rfidPlaceSet(QPoint p);
      void thermalPlaceSet(QPoint p);
      void co2PlaceSet(QPoint p);
      void updateMapInternal(void);
      void laserVisualizerClicked(QString robotName,QString laserName);
      void sonarVisualizerClicked(QString robotName,QString sonarName);
      void robotVisualizerClicked(QString robotName);
      void itemClicked(QPoint p,Qt::MouseButton b);
      void robotReplaceSet(QPoint p,std::string robotName);
      
    Q_SIGNALS:
      void waitForRobotPose(void);
      void waitForThermalPose(void);
      void waitForCo2Pose(void);
      void waitForRfidPose(void);
      void updateMap(void);
      void replaceRobot(std::string robotFrameId);
  };
}

#endif

