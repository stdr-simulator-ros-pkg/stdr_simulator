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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_co2.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  **/
  CGuiCO2::CGuiCO2(stdr_msgs::CO2SensorMsg msg,std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    tf_frame_ = baseTopic + "_" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiCO2::callback,this);
    visualization_status_ = 0;
  }
  
  /**
  @brief Callback for the rfid measurements message
  **/
  void CGuiCO2::callback(const stdr_msgs::CO2SensorMeasurementMsg& msg)
  {
    if(lock_)
    {
      return;
    }
    co2_sources_ = msg;
  }
  
  /**
  @brief Paints the sensor measurement in the map image
  **/
  void CGuiCO2::paint(
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
    
    //!< Draw measurement stuff
    //~ QBrush brush(QColor(50,100,50,75 * (2 - visualization_status_)));
    //~ painter.setBrush(brush);
    
    //~ for(unsigned int j = 0 ; j < co2_sources_.co2_sources_ids.size() ; j++)
    //~ {
      //~ for(unsigned int i = 0 ; i < env_co2_sources_.co2_sources.size() ; i++)
      //~ {
        //~ if(co2_sources_.co2_sources_ids[j] == env_co2_sources_.co2_sources[i].id)
        //~ {
          //~ int x1 = pose_x / ocgd;
          //~ int y1 = pose_y / ocgd;
          //~ int x2 = env_co2_sources_.co2_sources[i].pose.x / ocgd;
          //~ int y2 = env_co2_sources_.co2_sources[i].pose.y / ocgd;
          //~ painter.drawLine(x1, y1, x2, y2);
          //~ break;
        //~ }
      //~ }
    //~ }
    
    QBrush brush_cone(QColor(50,100,50, 20 * (2 - visualization_status_)));
    painter.setBrush(brush_cone);
    QPen pen(QColor(0,0,0,0));
    painter.setPen(pen);

    painter.drawPie(
      pose_x / ocgd - msg_.maxRange / ocgd,
        
      pose_y / ocgd - msg_.maxRange / ocgd,
      
      msg_.maxRange / ocgd * 2,
      msg_.maxRange / ocgd * 2,
      
      - 0,
        
      - 2 * 180.0 * 16);

    lock_ = false;
  }
  
  /**
  @brief Default destructor
  **/
  CGuiCO2::~CGuiCO2(void)
  {

  }
  
  /**
  @brief Returns the visibility status of the specific sensor
  **/
  char CGuiCO2::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status of the specific sensor
  **/
  void CGuiCO2::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
  }
  
  /**
  @brief Sets the visibility status of the specific sensor
  **/
  void CGuiCO2::setVisualizationStatus(char vs)
  {
    visualization_status_ = vs;
  }
  
  /**
  @brief Returns the frame id of the specific sensor
  @return std::string : The sensor's frame id
  **/
  std::string CGuiCO2::getFrameId(void)
  {
    return msg_.frame_id;
  }
  
  /**
  @brief Sets the tags existent in the environment
  @param env_tags [stdr_msgs::CO2SourceVector] The sources vector
  @return void
  **/
  void CGuiCO2::setEnvironmentalCO2Sources(
    stdr_msgs::CO2SourceVector env_co2_sources)
  {
    while(lock_)
    {
      usleep(100);
    }
    lock_ = true;
    env_co2_sources_ = env_co2_sources;
    lock_ = false;
  }
}
