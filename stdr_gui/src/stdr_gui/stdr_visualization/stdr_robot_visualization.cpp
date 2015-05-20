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

#include "stdr_gui/stdr_visualization/stdr_robot_visualization.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  @param name [QString] Robot frame id
  @param resolution [float] Map resolution
  @return void
  **/
  CRobotVisualisation::CRobotVisualisation(QString name,float resolution):
    name_(name),
    resolution_(resolution)
  {
    setupUi(this);
    setWindowTitle(name_);
    active_ = true;
    
    void_image_ = QImage(
      robotImage->width(),
      robotImage->height(),
      QImage::Format_RGB32);
      
    void_image_.fill(QColor(255,255,255,255));
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CRobotVisualisation::~CRobotVisualisation(void)
  {
    
  }
  
  /**
  @brief Destroys the visualizer
  @return void
  **/
  void CRobotVisualisation::destruct(void)
  {
    active_ = false;
    hide();
    delete robotImage;
    delete robotPose;
    delete robotSpeeds;
  }
  
  /**
  @brief Called when the close event is triggered
  @param event [QCloseEvent*] The close event
  @return void
  **/
  void CRobotVisualisation::closeEvent(QCloseEvent *event)
  {
    destruct();
    active_ = false;
  }
  
  /**
  @brief Returns true if the visualizer is active
  @return bool
  **/
  bool CRobotVisualisation::getActive(void)
  {
    return active_;
  }

  /**
  @brief Sets the image to be shown
  @param img [QImage] The drawn image
  @return void
  **/
  void CRobotVisualisation::setImage(QImage img)
  {
    internal_image_ = img;
    robotImage->setPixmap(
      QPixmap().fromImage(internal_image_.mirrored(false,true)));
  }
  
  /**
  @brief Sets the robot's current pose
  @param pose [geometry_msgs::Pose2D] the robot pose
  @return void
  **/
  void CRobotVisualisation::setCurrentPose(geometry_msgs::Pose2D pose)
  {
    robotPoseX->setText(
      QString("\tx = ") + QString().setNum(pose.x) + QString(" m"));
    robotPoseY->setText(
      QString("\ty = ") + QString().setNum(pose.y) + QString(" m"));
    robotPoseTheta->setText(
      QString("\ttheta = ") + QString().setNum(pose.theta) + QString(" rad"));
  }
  
  /**
  @brief Sets the robot's current speed
  @param msg [std::pair<float,float>] the robot speeds
  @return void
  **/
  void CRobotVisualisation::setCurrentSpeed(std::vector<float> msg)
  {
    robotSpeedLinear->setText(
      QString("\tu_x = ") + QString().setNum(msg[0]) + QString(" m/s"));
    robotSpeedLinearY->setText(
      QString("\tu_y = ") + QString().setNum(msg[1]) + QString(" m/s"));
    robotSpeedAngular->setText(
      QString("\tw = ") + QString().setNum(msg[2]) + QString(" rad/s"));
  }
}
