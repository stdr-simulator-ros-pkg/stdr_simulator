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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_bumper.h"

namespace stdr_gui{
  
  /**
  @brief Default contructor
  @param msg [stdr_msgs::BumperSensorMsg] The bumper description msg
  @param baseTopic [std::string] The ros topic for subscription
  @return void
  **/
  CGuiBumper::CGuiBumper(stdr_msgs::BumperSensorMsg msg, std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    tf_frame_ = baseTopic + "_" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiBumper::callback, this);
    visualization_status_ = 0;
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiBumper::~CGuiBumper(void)
  {

  }
  
  /**
  @brief Callback for the ros bumper message
  @param msg [const stdr_msgs::Bumper&] The new bumper message
  @return void
  **/
  void CGuiBumper::callback(const stdr_msgs::Bumper& msg)
  {
    if(lock_)
    {
      return;
    }
    bumper_ = msg;
  }
  
  /**
  @brief Paints the bumper in the map image
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param listener [tf::TransformListener *] ROS tf transform listener
  @return void
  **/
  void CGuiBumper::paint(
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
    
    //!< Draw bumper stuff
    QBrush brush(QColor(0,0,0,255));
    painter.setBrush(brush);
    if(bumper_.contact) {
	QPen pen(QColor(255,0,0,255));
	painter.setPen(pen);
    }else {
	QPen pen(QColor(0,0,255,255));
	painter.setPen(pen);
    }
    painter.drawArc(
	(pose_x - msg_.radius) / ocgd,
	(pose_y - msg_.radius) / ocgd,
	msg_.radius / ocgd * 2,
	msg_.radius / ocgd * 2,
	- (pose_theta - msg_.contactAngle / 2) * 180 / STDR_PI * 16,
	- msg_.contactAngle * 180 / STDR_PI * 16);
    lock_ = false;
  }
  
  /**
  @brief Paints the bumper range in it's own visualizer
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param maxRange [float] The maximum range of all the robot sensors. Used for the visualizer proportions 
  @return void
  **/
  void CGuiBumper::visualizerPaint(
    QImage *m,
    float ocgd,
    float maxRange)
  {
    float size = m->width();
    float climax = size / maxRange * ocgd / 2.1;
    lock_ = true;
    QPainter painter(m);
    
    //!< Draw bumper stuff
    if(bumper_.contact) {
	QBrush brush(QColor(255,0,0,255));
	painter.setBrush(brush);
	QPen pen(QColor(0,255,0,255));
	painter.setPen(pen);
	painter.drawArc(
	    size / 2 + msg_.pose.x / ocgd * climax,
	    size / 2 + msg_.pose.y / ocgd * climax,
	    msg_.radius / ocgd * 2 * climax,
	    msg_.radius / ocgd * 2 * climax,
	    (msg_.pose.theta + msg_.contactAngle) * 180 / STDR_PI / 2 * 16,
	    (msg_.pose.theta - msg_.contactAngle) * 180 / STDR_PI / 2 * 16);
    }      
    lock_ = false;
  }
  
  /**
  @brief Returns the visibility status of the specific bumper sensor
  @return char : The visibility status
  **/
  char CGuiBumper::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status of the specific bumper sensor
  @return void
  **/
  void CGuiBumper::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
  }
  
  /**
  @brief Returns the frame id of the specific bumper sensor
  @return std::string : The bumper frame id
  **/
  std::string CGuiBumper::getFrameId(void)
  {
    return msg_.frame_id;
  }
  
  /**
  @brief Sets the visibility status of the specific bumper sensor
  @param vs [char] The new visibility status
  @return void
  **/
  void CGuiBumper::setVisualizationStatus(char vs)
  {
    visualization_status_ = vs;
  }
}

