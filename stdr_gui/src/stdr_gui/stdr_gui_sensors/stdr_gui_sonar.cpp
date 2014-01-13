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
  
  /**
  @brief Default contructor
  @param msg [stdr_msgs::SonarSensorMsg] The sonar description msg
  @param baseTopic [std::string] The ros topic for subscription
  @return void
  **/
  CGuiSonar::CGuiSonar(stdr_msgs::SonarSensorMsg msg,std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    tf_frame_ = baseTopic + "_" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiSonar::callback,this);
    visualization_status_ = 0;
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiSonar::~CGuiSonar(void)
  {

  }
  
  /**
  @brief Callback for the ros sonar message
  @param msg [const sensor_msgs::Range&] The new sonar range message
  @return void
  **/
  void CGuiSonar::callback(const sensor_msgs::Range& msg)
  {
    if(lock_)
    {
      return;
    }
    range_ = msg;
  }
  
  /**
  @brief Paints the sonar range in the map image
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param listener [tf::TransformListener *] ROS tf transform listener
  @return void
  **/
  void CGuiSonar::paint(
    QImage *m,
    float ocgd,
    tf::TransformListener *listener)
  {
    lock_ = true;
    QPainter painter(m);
    
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
    
    float real_dist = range_.range;
    if(real_dist > msg_.maxRange)
    {
      real_dist = msg_.maxRange;
      QBrush brush(QColor(100,100,100,75 * (2 - visualization_status_)));
      painter.setBrush(brush);
      QPen pen(QColor(0,0,0,0));
      painter.setPen(pen);
    }
    else if(real_dist < msg_.minRange)
    {
      real_dist = msg_.minRange;
      QBrush brush(QColor(100,100,100,75 * (2 - visualization_status_)));
      painter.setBrush(brush);
      QPen pen(QColor(0,0,0,0));
      painter.setPen(pen);
    }
    else
    {
      QBrush brush(QColor(0,200,0,75 * (2 - visualization_status_)));
      painter.setBrush(brush);
      QPen pen(QColor(0,0,0,0));
      painter.setPen(pen);
    }
    
    painter.drawPie(
      pose_x / ocgd - real_dist / ocgd,
        
      pose_y / ocgd - real_dist / ocgd,
      
      real_dist / ocgd * 2,
      real_dist / ocgd * 2,
      
      - (pose_theta - msg_.coneAngle / 2.0)
        * 180.0 / STDR_PI * 16,
        
      - msg_.coneAngle * 180.0 / STDR_PI * 16);

    lock_ = false;
  }
  
  /**
  @brief Paints the sonar range in it's own visualizer
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param maxRange [float] The maximum range of all the robot sensors. Used for the visualizer proportions 
  @return void
  **/
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
  
  /**
  @brief Returns the max range of the specific sonar sensor
  @return float
  **/
  float CGuiSonar::getMaxRange(void)
  {
    return msg_.maxRange;
  }
  
  /**
  @brief Returns the visibility status of the specific sonar sensor
  @return char : The visibility status
  **/
  char CGuiSonar::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status of the specific sonar sensor
  @return void
  **/
  void CGuiSonar::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
  }
  
  /**
  @brief Returns the frame id of the specific sonar sensor
  @return std::string : The sonar frame id
  **/
  std::string CGuiSonar::getFrameId(void)
  {
    return msg_.frame_id;
  }
  
  /**
  @brief Sets the visibility status of the specific sonar sensor
  @param vs [char] The new visibility status
  @return void
  **/
  void CGuiSonar::setVisualizationStatus(char vs)
  {
    visualization_status_ = vs;
  }
}

