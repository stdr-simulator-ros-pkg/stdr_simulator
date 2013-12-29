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
  /** 
  @brief Default contructor
  @param name [QString] Sonar frame id
  @param resolution [float] Map resolution
  @return void
  **/
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
  
  /**
  @brief Default destructor
  @return void
  **/
  CSonarVisualisation::~CSonarVisualisation(void)
  {
    
  }
  
  /**
  @brief Destroys the visualizer
  @return void
  **/
  void CSonarVisualisation::destruct(void)
  {
    hide();
    delete sonarDistBar;
    delete sonarDist;
    delete sonarMaxDist;
    delete sonarMinDist;
  }
  
  /**
  @brief Called when the close event is triggered
  @param event [QCloseEvent*] The close event
  @return void
  **/
  void CSonarVisualisation::closeEvent(QCloseEvent *event)
  {
    destruct();
    active_ = false;
    subscriber_.shutdown();
  }
  
  /**
  @brief Returns true if the visualizer is active
  @return bool
  **/
  bool CSonarVisualisation::getActive(void)
  {
    return active_;
  }
  
  /**
  @brief Sets the sonar description message
  @param msg [stdr_msgs::SonarSensorMsg] The sonar description
  @return void
  **/
  void CSonarVisualisation::setSonar(stdr_msgs::SonarSensorMsg msg)
  {
    msg_ = msg;
    sonarMaxDist->setText(QString().setNum(msg.maxRange) + QString(" m"));
    sonarMinDist->setText(QString().setNum(msg.minRange) + QString(" m"));
  }
  
  /**
  @brief Called when new laser data are available
  @param msg [const sensor_msgs::Range&] The new sonar data
  @return void
  **/
  void CSonarVisualisation::callback(const sensor_msgs::Range& msg)
  {
    range_ = msg;
  }
  
  /**
  @brief Paints the visualizer
  @return void
  **/
  void CSonarVisualisation::paint(void)
  {
	  
	float real_dist = range_.range;
    if(real_dist > msg_.maxRange)
    {
      real_dist = msg_.maxRange;
    }
    else if(real_dist < msg_.minRange)
    {
      real_dist = msg_.minRange;
    }
      
    sonarDist->setText(QString().setNum(real_dist) + QString(" m"));
    sonarDistBar->setValue(sonarDistBar->maximum() * real_dist / msg_.maxRange);
  }
}
