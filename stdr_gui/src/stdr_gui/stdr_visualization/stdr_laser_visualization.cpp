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
  
  CLaserVisualisation::~CLaserVisualisation(void)
  {
    
  }
  
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
  
  void CLaserVisualisation::closeEvent(QCloseEvent *event)
  {
    destruct();
    active_ = false;
    subscriber_.shutdown();
  }
  
  bool CLaserVisualisation::getActive(void)
  {
    return active_;
  }
  
  void CLaserVisualisation::setLaser(stdr_msgs::LaserSensorMsg msg)
  {
    msg_ = msg;
    laserMax->setText(QString().setNum(msg.maxRange) + QString(" m"));
    laserMin->setText(QString().setNum(msg.minRange) + QString(" m"));
  }
  
  void CLaserVisualisation::callback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;
  }
  
  void CLaserVisualisation::paint(void)
  {
    internal_image_ = void_image_;
    QPainter painter(&internal_image_);
    painter.setPen(QColor(255,0,0,255));
    float mean = 0;
    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      mean += scan_.ranges[i];
      painter.drawLine(
        internal_image_.width() / 2,
        internal_image_.height() / 2,
        internal_image_.width() / 2 + scan_.ranges[i] / msg_.maxRange * 
            cos(scan_.angle_min + ((float)i) * scan_.angle_increment) *
            internal_image_.width() / 2,
        internal_image_.height() / 2 + scan_.ranges[i] / msg_.maxRange *
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
