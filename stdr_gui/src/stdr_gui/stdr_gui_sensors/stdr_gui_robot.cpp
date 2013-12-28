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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  @param msg [const stdr_msgs::RobotIndexedMsg&] The robot description msg
  @return void
  **/
  CGuiRobot::CGuiRobot(const stdr_msgs::RobotIndexedMsg& msg)
  {
    robot_initialized_ = false;
    initial_pose_ = msg.robot.initialPose;
    current_pose_ = initial_pose_;
    footprint_ = msg.robot.footprint;
    radius_ = msg.robot.footprint.radius;
    frame_id_ = msg.name;
    show_label_ = true;
    show_circles_ = false;
    visualization_status_ = 0;
    for(unsigned int i = 0 ; i < msg.robot.laserSensors.size() ; i++)
    {
      CGuiLaser *l = new CGuiLaser(msg.robot.laserSensors[i], frame_id_);
      lasers_.push_back(l);
    }
    for(unsigned int i = 0 ; i < msg.robot.sonarSensors.size() ; i++)
    {
      CGuiSonar *l = new CGuiSonar(msg.robot.sonarSensors[i], frame_id_);
      sonars_.push_back(l);
    }
    robot_initialized_ = true;
  }
  
  /**
  @brief Paints the robot and it's sensors to the image
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param listener [tf::TransformListener *] ROS tf listener to get the robot's current pose
  @return void
  **/
  void CGuiRobot::draw(QImage *m,float ocgd,tf::TransformListener *listener)
  {
    if(!robot_initialized_)
    {
      return;
    }
    resolution_ = ocgd;
    tf::StampedTransform transform;
      
    try
    {
      listener->lookupTransform("map", 
        frame_id_.c_str(), ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_DEBUG("%s",ex.what());
    }
    tfScalar roll,pitch,yaw;
    current_pose_.x = transform.getOrigin().x();
    current_pose_.y = transform.getOrigin().y();
    transform.getBasis().getRPY(roll,pitch,yaw);
    current_pose_.theta = yaw;
    
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      lasers_[i]->paint(m,resolution_,current_pose_);
    }
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      sonars_[i]->paint(m,resolution_,current_pose_);
    }
    
    drawSelf(m);
  }
  
  /**
  @brief Draws the robot body 
  @param m [QImage*] The image for the robot to draw itself
  @return void
  **/
  void CGuiRobot::drawSelf(QImage *m)
  {
    QPainter painter(m);
    painter.setPen(QColor(0,0,200,10 + 100 * (2 - visualization_status_)));
    
    painter.drawEllipse(
      (current_pose_.x - radius_) / resolution_,
      (current_pose_.y - radius_) / resolution_,
      radius_ * 2.0 / resolution_,
      radius_ * 2.0 / resolution_);
    
    painter.drawLine(	
      current_pose_.x / resolution_,
      current_pose_.y / resolution_,
      current_pose_.x / resolution_ + 
        radius_ / resolution_ * 1.05 * cos(current_pose_.theta),
      current_pose_.y / resolution_ + 
        radius_ / resolution_ * 1.05 * sin(current_pose_.theta));
  
    if(show_circles_)
    {
      painter.setPen(QColor(255,0,0,10 + 100 * (2 - visualization_status_)));
      for(unsigned int i = 0 ; i < 5 ; i++)
      {
        painter.drawEllipse(
          (current_pose_.x - (i + 1.0) / 2.0) / resolution_,
          (current_pose_.y - (i + 1.0) / 2.0) / resolution_,
          (i + 1.0) / resolution_,
          (i + 1.0) / resolution_);
      }
    }
  }
  
  /**
  @brief Checks if the robot is near a specific point
  @param p [QPoint] A point
  @return bool : True if the robot is in proximity with p
  **/
  bool CGuiRobot::checkEventProximity(QPoint p)
  {
    float dx = p.x() * resolution_ - current_pose_.x;
    float dy = p.y() * resolution_ - current_pose_.y;
    float dist = sqrt( pow(dx,2) + pow(dy,2) );
    return dist <= radius_;
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiRobot::~CGuiRobot(void)
  {
    
  }
  
  /**
  @brief Destroys the robot object
  @return void
  **/
  void CGuiRobot::destroy(void){
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      delete lasers_[i];
    }
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      delete sonars_[i];
    }
  }

  /**
  @brief Returns the frame id of the specific robot
  @return std::string : The robot frame id
  **/
  std::string CGuiRobot::getFrameId(void)
  {
    return frame_id_;
  }
  
  /**
  @brief Draws the robot's label
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @return void
  **/
  void CGuiRobot::drawLabel(QImage *m,float ocgd)
  {
    QPainter painter(m);
    
    painter.setPen(QColor(0,0,0,10 + 100 * (2 - visualization_status_)));
    
    painter.drawRect(
      current_pose_.x / ocgd + 10,
      m->height() - (current_pose_.y / ocgd) - 30,
      100,
      20);
    
    painter.setPen(QColor(255,255,255,10 + 100 * (2 - visualization_status_)));
    
    painter.fillRect(
      current_pose_.x / ocgd + 10,
      m->height() - (current_pose_.y / ocgd) - 30,
      100,
      20,
      QBrush(QColor(0,0,0,10 + 100 * (2 - visualization_status_))));
    
    painter.drawText(
      current_pose_.x / ocgd + 12,
      m->height() - (current_pose_.y / ocgd) - 15,
      QString(frame_id_.c_str()));
  }
  
  /**
  @brief Sets the show_label_ flag
  @param b [bool] True for showing the label
  @return void
  **/
  void CGuiRobot::setShowLabel(bool b)
  {
    show_label_ = b;
  }
  
  /**
  @brief Gets the show_label_ flag
  @return bool : show_label_
  **/
  bool CGuiRobot::getShowLabel(void)
  {
    return show_label_;
  }
  
  /**
  @brief Gets the show_label_ flag
  @return bool : show_label_
  **/
  void CGuiRobot::toggleShowLabel(void)
  {
    show_label_ =! show_label_;
  }
  
  /**
  @brief Toggles the show_circles_ flag
  @return void
  **/
  void CGuiRobot::toggleShowCircles(void)
  {
    show_circles_ =! show_circles_;
  }
  
  /**
  @brief Returns the current robot pose
  @return QPoint : The current robot pose
  **/
  QPoint CGuiRobot::getCurrentPose(void)
  {
    return QPoint(current_pose_.x / resolution_, current_pose_.y / resolution_);
  }
  
  /**
  @brief Returns the lasers number
  @return int : the lasers number
  **/
  int CGuiRobot::getLasersNumber(void)
  {
    return lasers_.size();
  }
  
  /**
  @brief Returns the sonars number
  @return int : the sonars number
  **/
  int CGuiRobot::getSonarsNumber(void)
  {
    return sonars_.size();
  }
  
  /**
  @brief Returns the visibility status
  @return char
  **/
  QImage CGuiRobot::getVisualization(float ocgd)
  {
    float maxRange = -1;
    for(unsigned int l = 0 ; l < lasers_.size() ; l++)
    {
      float t = lasers_[l]->getMaxRange();
      if(t > maxRange)
      {
        maxRange = t;
      }
    }
    for(unsigned int l = 0 ; l < sonars_.size() ; l++)
    {
      float t = sonars_[l]->getMaxRange();
      if(t > maxRange)
      {
        maxRange = t;
      }
    }
    visualization = QImage(310,310,QImage::Format_RGB32);
    visualization.fill(Qt::white);
    for(unsigned int l = 0 ; l < lasers_.size() ; l++)
    {
      lasers_[l]->visualizerPaint(&visualization,ocgd,maxRange);
    }
    for(unsigned int l = 0 ; l < sonars_.size() ; l++)
    {
      sonars_[l]->visualizerPaint(&visualization,ocgd,maxRange);
    }
    return visualization;
  }
  
  /**
  @brief Returns the laser visibility status
  @param frame_id [std::string] The laser frame id
  @return char
  **/
  char CGuiRobot::getLaserVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      if(lasers_[i]->getFrameId() == frame_id)
      {
        return lasers_[i]->getVisualizationStatus();
      }
    }
  }
    
  /**
  @brief Toggles the laser visibility status
  @param frame_id [std::string] The laser frame id
  @return void
  **/
  void CGuiRobot::toggleLaserVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      if(lasers_[i]->getFrameId() == frame_id)
      {
        lasers_[i]->toggleVisualizationStatus();
      }
    }
  }
  
  /**
  @brief Returns the sonar visibility status
  @param frame_id [std::string] The sonar frame id
  @return char
  **/
  char CGuiRobot::getSonarVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      if(sonars_[i]->getFrameId() == frame_id)
      {
        return sonars_[i]->getVisualizationStatus();
      }
    }
  }
  
  /**
  @brief Toggles the sonar visibility status
  @param frame_id [std::string] The sonar frame id
  @return void
  **/
  void CGuiRobot::toggleSonarVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      if(sonars_[i]->getFrameId() == frame_id)
      {
        sonars_[i]->toggleVisualizationStatus();
      }
    }
  }
  
  /**
  @brief Returns the visibility status
  @return char
  **/
  char CGuiRobot::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status
  @return void
  **/
  void CGuiRobot::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      lasers_[i]->setVisualizationStatus(visualization_status_);
    }
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      sonars_[i]->setVisualizationStatus(visualization_status_);
    }
  }
}
