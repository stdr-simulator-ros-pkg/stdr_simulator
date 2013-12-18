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

#ifndef STDR_GUI_ROBOT_CONTAINER
#define STDR_GUI_ROBOT_CONTAINER

#include "stdr_gui/stdr_tools.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_laser.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_rfid.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_sonar.h"

namespace stdr_gui
{
  
  class CGuiRobot
  {
    private:
      bool show_label_;
      bool show_circles_;
      bool robot_initialized_;
      
      float radius_;
      float resolution_;
      
      std::vector<CGuiLaser*>   lasers_;
      std::vector<CGuiSonar*>   sonars_;
      std::vector<CGuiRfid*>     rfids_;
      
      std::string frame_id_;
      
      geometry_msgs::Pose2D initial_pose_;
      geometry_msgs::Pose2D current_pose_;
      
      stdr_msgs::FootprintMsg footprint_;
      
      QImage visualization;
      
      void drawSelf(QImage *m);
      
    public:
      CGuiRobot(const stdr_msgs::RobotIndexedMsg& msg);
      ~CGuiRobot(void);
      
      std::string getFrameId(void);
      void draw(QImage *m,float ocgd,tf::TransformListener *_listener);
      void drawLabel(QImage *m,float ocgd);
      bool checkEventProximity(QPoint p);
      void setShowLabel(bool b);
      void toggleShowLabel(void);
      bool getShowLabel(void);
      void toggleShowCircles(void);
      QPoint getCurrentPose(void);
      void destroy(void);
      int getLasersNumber(void);
      int getSonarsNumber(void);
      
      QImage getVisualization(float ocgd);
  };  
}

#endif
