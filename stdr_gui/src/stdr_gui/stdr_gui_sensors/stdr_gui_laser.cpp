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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_laser.h"

namespace stdr_gui
{
  
  CGuiLaser::CGuiLaser(stdr_msgs::LaserSensorMsg msg,std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiLaser::callback,this);
    visualization_status_ = 0;
  }
  
  CGuiLaser::~CGuiLaser(void)
  { 
  }
  
  char CGuiLaser::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  void CGuiLaser::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
  }
  
  void CGuiLaser::callback(const sensor_msgs::LaserScan& msg)
  {
    if(lock_)
    {
      return;
    }
    scan_ = msg;
  }
  
  void CGuiLaser::paint(
    QImage *m,
    float ocgd,
    geometry_msgs::Pose2D robotPose)
  {
    lock_ = true;
    QPainter painter(m);
    
    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i];
      if(real_dist > msg_.maxRange)
      {
        real_dist = msg_.maxRange;
        painter.setPen(QColor(255,0,0,100));
      }
      else if(real_dist < msg_.minRange)
      {
        real_dist = msg_.minRange;
        painter.setPen(QColor(100,100,100,100));
      }
      else
      {
        painter.setPen(QColor(255,0,0,100));
      }
      painter.drawLine(
        robotPose.x / ocgd + (msg_.pose.x / ocgd * cos(robotPose.theta) - 
          msg_.pose.y / ocgd * sin(robotPose.theta)),
        
        robotPose.y / ocgd + (msg_.pose.x / ocgd * sin(robotPose.theta) + 
          msg_.pose.y / ocgd * cos(robotPose.theta)),
          
        robotPose.x / ocgd + (msg_.pose.x / ocgd * cos(robotPose.theta) - 
          msg_.pose.y /ocgd * sin(robotPose.theta)) + real_dist * 
          cos(robotPose.theta + scan_.angle_min + i * scan_.angle_increment)
           / ocgd,
          
        robotPose.y / ocgd + (msg_.pose.x / ocgd * sin(robotPose.theta) + 
          msg_.pose.y / ocgd * cos(robotPose.theta)) + real_dist *
          sin(robotPose.theta + scan_.angle_min + i * scan_.angle_increment)
           / ocgd
      );
    }
    lock_ = false;
  }
  
  void CGuiLaser::visualizerPaint(
    QImage *m,
    float ocgd,
    float maxRange)
  {
    lock_ = true;
    QPainter painter(m);
    float size = m->width();
    float climax = size / maxRange * ocgd / 2.1;
    
    
    
    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
	  float real_dist = scan_.ranges[i];
      if(real_dist > msg_.maxRange)
      {
        real_dist = msg_.maxRange;
        painter.setPen(QColor(255,0,0,100));
      }
      else if(real_dist < msg_.minRange)
      {
        real_dist = msg_.minRange;
        painter.setPen(QColor(100,100,100,100));
      }
      else
      {
        painter.setPen(QColor(255,0,0,100));
      }
		
      painter.drawLine(
        size / 2 + (msg_.pose.x / ocgd) * climax,
        size / 2 + (msg_.pose.y / ocgd) * climax,

        size / 2 + ((msg_.pose.x / ocgd) + real_dist *
          cos(scan_.angle_min + i * scan_.angle_increment) / ocgd) * climax,
        size / 2 + ((msg_.pose.y / ocgd) + real_dist * 
          sin(scan_.angle_min + i * scan_.angle_increment) / ocgd) * climax
      );
    }
    lock_ = false;
  }
  
  float CGuiLaser::getMaxRange(void)
  {
    return msg_.maxRange;
  }
  
  std::string CGuiLaser::getFrameId(void)
  {
    return msg_.frame_id;
  }
}

