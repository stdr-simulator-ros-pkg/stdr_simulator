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

#include "stdr_gui/stdr_visualization/stdr_sonar_visualization.h"

namespace stdr_gui
{

  CSonarVisualisation::CSonarVisualisation(QString name,float resolution):
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
      &CSonarVisualisation::callback,
      this);
      
    
  }
  
  CSonarVisualisation::~CSonarVisualisation(void)
  {
    
  }
  
  void CSonarVisualisation::destruct(void)
  {
    hide();
    delete sonarDistBar;
    delete sonarDist;
    delete sonarMaxDist;
    delete sonarMinDist;
  }
  
  void CSonarVisualisation::closeEvent(QCloseEvent *event)
  {
    destruct();
    active_ = false;
    subscriber_.shutdown();
  }
  
  bool CSonarVisualisation::getActive(void)
  {
    return active_;
  }
  
  void CSonarVisualisation::setSonar(stdr_msgs::SonarSensorMsg msg)
  {
    msg_ = msg;
    sonarMaxDist->setText(QString().setNum(msg.maxRange) + QString(" m"));
    sonarMinDist->setText(QString().setNum(msg.minRange) + QString(" m"));
  }
  
  void CSonarVisualisation::callback(const sensor_msgs::Range& msg)
  {
    range_ = msg;
  }
  
  void CSonarVisualisation::paint(void)
  {
    sonarDist->setText(QString().setNum(range_.range) + QString(" m"));
    sonarDistBar->setValue(
      sonarDistBar->maximum() * range_.range/msg_.maxRange);
  }
}
