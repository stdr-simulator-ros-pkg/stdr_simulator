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
  /**
  @brief Default contructor
  @param msg [stdr_msgs::LaserSensorMsg] The laser description msg
  @param baseTopic [std::string] The ros topic for subscription
  @return void
  **/
  CGuiLaser::CGuiLaser(stdr_msgs::LaserSensorMsg msg,std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    tf_frame_ = baseTopic + "_" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiLaser::callback,this);
    visualization_status_ = 0;
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiLaser::~CGuiLaser(void)
  { 
  }
  
  /**
  @brief Returns the visibility status of the specific laser sensor
  @return void
  **/
  char CGuiLaser::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status of the specific laser sensor
  @return void
  **/
  void CGuiLaser::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
  }
  
  /**
  @brief Callback for the ros laser message
  @param msg [const sensor_msgs::LaserScan&] The new laser scan message
  @return void
  **/
  void CGuiLaser::callback(const sensor_msgs::LaserScan& msg)
  {
    if(lock_)
    {
      return;
    }
    scan_ = msg;
  }
  
  /**
  @brief Paints the laser scan in the map image
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param listener [tf::TransformListener *] ROS tf transform listener
  @return void
  **/
  void CGuiLaser::paint(
    QImage *m,
    float ocgd,
    tf::TransformListener *listener)
  {
    lock_ = true;
    QPainter painter(m);
    //~ painter.setRenderHint(QPainter::Antialiasing, true);
    
    //!< Find transformation
    tf::StampedTransform transform;
      
    try
    {
      listener->waitForTransform("map_static",
                                  tf_frame_.c_str(),
                                  ros::Time(0),
                                  ros::Duration(0.2));
      listener->lookupTransform("map_static", 
        tf_frame_.c_str(), ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_DEBUG("%s",ex.what());
    }
    tfScalar roll,pitch,yaw;
    float pose_x = transform.getOrigin().x();
    float pose_y = transform.getOrigin().y();
    transform.getBasis().getRPY(roll,pitch,yaw);
    float pose_theta = yaw;
    
    //!< Draw laser stuff
    
    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i];
      if(real_dist > msg_.maxRange)
      {
        real_dist = msg_.maxRange;
        painter.setPen(QColor(255,0,0,75 * (2 - visualization_status_)));
      }
      else if(real_dist < msg_.minRange)
      {
        real_dist = msg_.minRange;
        painter.setPen(QColor(100,100,100,
          75 * (2 - visualization_status_)));
      }
      else
      {
        painter.setPen(QColor(255,0,0,75 * (2 - visualization_status_)));
      }
      painter.drawLine(
        pose_x / ocgd,
        
        pose_y / ocgd,
          
        pose_x / ocgd + real_dist * 
          cos(pose_theta + scan_.angle_min + i * scan_.angle_increment)
           / ocgd,
          
        pose_y / ocgd + real_dist * 
          sin(pose_theta + scan_.angle_min + i * scan_.angle_increment)
           / ocgd
      );
    }
    lock_ = false;
  }
  
  /**
  @brief Paints the laser scan in it's own visualizer
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param maxRange [float] The maximum range of all the robot sensors. Used for the visualizer proportions 
  @return void
  **/
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
          cos(scan_.angle_min + i * scan_.angle_increment + msg_.pose.theta)
          / ocgd) * climax,
        size / 2 + ((msg_.pose.y / ocgd) + real_dist * 
          sin(scan_.angle_min + i * scan_.angle_increment + msg_.pose.theta) 
          / ocgd) * climax
      );
    }
    lock_ = false;
  }
  
  /**
  @brief Returns the max range of the specific laser sensor
  @return void
  **/
  float CGuiLaser::getMaxRange(void)
  {
    return msg_.maxRange;
  }
  
  /**
  @brief Returns the frame id of the specific laser sensor
  @return std::string : The laser frame id
  **/
  std::string CGuiLaser::getFrameId(void)
  {
    return msg_.frame_id;
  }
  
  /**
  @brief Sets the visibility status of the specific laser sensor
  @param vs [char] The new visibility status
  @return void
  **/
  void CGuiLaser::setVisualizationStatus(char vs)
  {
    visualization_status_ = vs;
  }
}

