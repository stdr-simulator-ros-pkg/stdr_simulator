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

#include "stdr_gui/stdr_visualization/stdr_laser_visualization.h"

namespace stdr_gui
{

  /**
  @brief Default contructor
  @param name [QString] Laser frame id
  @param resolution [float] Map resolution
  @return void
  **/
  CLaserVisualisation::CLaserVisualisation(QString name,float resolution):
    name_(name),
    resolution_(resolution)
  {
    setupUi(this);
    setWindowTitle(name_);
    active_ = true;
    
    ros::NodeHandle n;
    
    subscriber_ = n.subscribe(
      name_.toStdString().c_str(), 
      1, 
      &CLaserVisualisation::callback,
      this);
    
    void_image_ = QImage(
      laserImage->width(),
      laserImage->height(),
      QImage::Format_RGB32);
      
    void_image_.fill(QColor(255,255,255,255));
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CLaserVisualisation::~CLaserVisualisation(void)
  {
    
  }
  
  /**
  @brief Destroys the visualizer
  @return void
  **/
  void CLaserVisualisation::destruct(void)
  {
    active_ = false;
    subscriber_.shutdown();
    hide();
    delete laserMean;
    delete laserMax;
    delete laserMin;
    delete laserImage;
  }
  
  /**
  @brief Called when the close event is triggered
  @param event [QCloseEvent*] The close event
  @return void
  **/
  void CLaserVisualisation::closeEvent(QCloseEvent *event)
  {
    destruct();
    active_ = false;
    subscriber_.shutdown();
  }
  
  /**
  @brief Returns true if the visualizer is active
  @return bool
  **/
  bool CLaserVisualisation::getActive(void)
  {
    return active_;
  }
  
  /**
  @brief Sets the laser description message
  @param msg [stdr_msgs::LaserSensorMsg] The laser description
  @return void
  **/
  void CLaserVisualisation::setLaser(stdr_msgs::LaserSensorMsg msg)
  {
    msg_ = msg;
    laserMax->setText(QString().setNum(msg.maxRange) + QString(" m"));
    laserMin->setText(QString().setNum(msg.minRange) + QString(" m"));
  }
  
  /**
  @brief Called when new laser data are available
  @param msg [const sensor_msgs::LaserScan&] The new laser data
  @return void
  **/
  void CLaserVisualisation::callback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;
  }
  
  /**
  @brief Paints the visualizer
  @return void
  **/
  void CLaserVisualisation::paint(void)
  {
    internal_image_ = void_image_;
    QPainter painter(&internal_image_);
    float mean = 0;
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
      
      mean += real_dist;
      painter.drawLine(
        internal_image_.width() / 2,
        internal_image_.height() / 2,
        internal_image_.width() / 2 + real_dist / msg_.maxRange * 
            cos(scan_.angle_min + ((float)i) * scan_.angle_increment) *
            internal_image_.width() / 2,
        internal_image_.height() / 2 + real_dist / msg_.maxRange *
            sin(scan_.angle_min + ((float)i) * scan_.angle_increment) *
            internal_image_.width() / 2
      );        
    }
    laserMean->setText(
      QString().setNum(mean/scan_.ranges.size()) + QString(" m"));
    laserImage->setPixmap(
      QPixmap().fromImage(internal_image_.mirrored(false,true)));
  }
}
