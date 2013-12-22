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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_sonar.h"

namespace stdr_gui{
  CGuiSonar::CGuiSonar(stdr_msgs::SonarSensorMsg msg,std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiSonar::callback,this);
  }
  
  CGuiSonar::~CGuiSonar(void)
  {

  }
  
  void CGuiSonar::callback(const sensor_msgs::Range& msg)
  {
    if(lock_)
    {
      return;
    }
    range_ = msg;
  }
  
  void CGuiSonar::paint(
    QImage *m,
    float ocgd,
    geometry_msgs::Pose2D robotPose)
  {
    lock_ = true;
    QPainter painter(m);
    
    
    float real_dist = range_.range;
    if(real_dist > msg_.maxRange)
    {
      real_dist = msg_.maxRange;
      QBrush brush(QColor(100,100,100,100));
      painter.setBrush(brush);
    }
    else if(real_dist < msg_.minRange)
    {
      real_dist = msg_.minRange;
      QBrush brush(QColor(100,100,100,100));
      painter.setBrush(brush);
    }
    else
    {
      QBrush brush(QColor(0,200,0,100));
      painter.setBrush(brush);
    }
    
    painter.drawPie(
      robotPose.x / ocgd +
        (msg_.pose.x / ocgd * cos(robotPose.theta) - 
        msg_.pose.y / ocgd * sin(robotPose.theta)) - 
        real_dist / ocgd,
        
      robotPose.y / ocgd +
        (msg_.pose.x / ocgd * sin(robotPose.theta) + 
        msg_.pose.y / ocgd * cos(robotPose.theta)) -
        real_dist / ocgd,
      
      real_dist / ocgd * 2,
      real_dist / ocgd * 2,
      
      - (robotPose.theta + msg_.pose.theta - msg_.coneAngle / 2.0)
        * 180.0 / STDR_PI * 16,
        
      - msg_.coneAngle * 180.0 / STDR_PI * 16);

    lock_ = false;
  }
  
  void CGuiSonar::visualizerPaint(
    QImage *m,
    float ocgd,
    float maxRange)
  {
    float size = m->width();
    float climax = size / maxRange * ocgd / 2.1;
    lock_ = true;
    QPainter painter(m);
    
    float real_dist = range_.range;
    if(real_dist > msg_.maxRange)
    {
      real_dist = msg_.maxRange;
      QBrush brush(QColor(100,100,100,100));
      painter.setBrush(brush);
    }
    else if(real_dist < msg_.minRange)
    {
      real_dist = msg_.minRange;
      QBrush brush(QColor(100,100,100,100));
      painter.setBrush(brush);
    }
    else
    {
      QBrush brush(QColor(0,200,0,100));
      painter.setBrush(brush);
    }
    
    painter.drawPie(
      size / 2 + (msg_.pose.x / ocgd - real_dist / ocgd) * climax,
      size / 2 + (msg_.pose.y / ocgd - real_dist / ocgd) * climax,
      
      real_dist / ocgd * 2 * climax,
      real_dist / ocgd * 2 * climax,
      
      -(msg_.pose.theta - msg_.coneAngle / 2.0) * 180.0 / STDR_PI * 16,
      
      -(msg_.coneAngle * 180.0 / STDR_PI) * 16);

    lock_ = false;
  }
  
  float CGuiSonar::getMaxRange(void)
  {
    return msg_.maxRange;
  }
}

