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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_sound.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  **/
  CGuiSound::CGuiSound(stdr_msgs::SoundSensorMsg msg,std::string baseTopic):
    msg_(msg)
  {
    topic_ = baseTopic + "/" + msg_.frame_id;
    tf_frame_ = baseTopic + "_" + msg_.frame_id;
    ros::NodeHandle n;
    lock_ = false;
    subscriber_ = n.subscribe(topic_.c_str(), 1, &CGuiSound::callback,this);
    visualization_status_ = 0;
  }
  
  /**
  @brief Callback for the rfid measurements message
  **/
  void CGuiSound::callback(const stdr_msgs::SoundSensorMeasurementMsg& msg)
  {
    if(lock_)
    {
      return;
    }
    sound_sources_ = msg;
  }
  
  /**
  @brief Paints the sensor measurement in the map image
  **/
  void CGuiSound::paint(
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
    QBrush brush(QColor(50,100,50,75 * (2 - visualization_status_)));
    painter.setBrush(brush);
    
    
    // What to draw?
    
    //~ for(unsigned int j = 0 ; j < sound_sources_.sound_source_ids.size() ; j++)
    //~ {
      //~ for(unsigned int i = 0 ; i < env_sound_sources_.sound_sources.size() ; i++)
      //~ {
        //~ if(sound_sources_.sound_sources_ids[j] == env_sound_sources_.sound_sources[i].id)
        //~ {
          //~ int x1 = pose_x / ocgd;
          //~ int y1 = pose_y / ocgd;
          //~ int x2 = env_sound_sources_.sound_sources[i].pose.x / ocgd;
          //~ int y2 = env_sound_sources_.sound_sources[i].pose.y / ocgd;
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
      
      - (pose_theta - msg_.angleSpan / 2.0)
        * 180.0 / STDR_PI * 16,
        
      - msg_.angleSpan * 180.0 / STDR_PI * 16);

    lock_ = false;
  }
  
  /**
  @brief Default destructor
  **/
  CGuiSound::~CGuiSound(void)
  {

  }
  
  /**
  @brief Returns the visibility status of the specific sensor
  **/
  char CGuiSound::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status of the specific sensor
  **/
  void CGuiSound::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
  }
  
  /**
  @brief Sets the visibility status of the specific sensor
  **/
  void CGuiSound::setVisualizationStatus(char vs)
  {
    visualization_status_ = vs;
  }
  
  /**
  @brief Returns the frame id of the specific sensor
  @return std::string : The sensor's frame id
  **/
  std::string CGuiSound::getFrameId(void)
  {
    return msg_.frame_id;
  }
  
  /**
  @brief Sets the tags existent in the environment
  @param env_tags [stdr_msgs::CO2SourceVector] The sources vector
  @return void
  **/
  void CGuiSound::setEnvironmentalSoundSources(
    stdr_msgs::SoundSourceVector env_sound_sources)
  {
    while(lock_)
    {
      usleep(100);
    }
    lock_ = true;
    env_sound_sources_ = env_sound_sources;
    lock_ = false;
  }
}
