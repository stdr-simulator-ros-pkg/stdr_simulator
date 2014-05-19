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

#include <stdr_robot/sensors/thermal.h>

namespace stdr_robot {
  
  /**
  @brief Default constructor
  **/ 
  ThermalSensor::ThermalSensor(
    const nav_msgs::OccupancyGrid& map,
    const stdr_msgs::ThermalSensorMsg& msg, 
    const std::string& name,
    ros::NodeHandle& n)
    : Sensor(map, name, n, msg.pose, msg.frame_id, msg.frequency)
  {
    _description = msg;

    _publisher = n.advertise<stdr_msgs::ThermalSensorMeasurementMsg>
      ( _namespace + "/" + msg.frame_id, 1 );
      
    thermal_sources_subscriber_ = n.subscribe(
      "stdr_server/thermal_sources_list", 
      1, 
      &ThermalSensor::receiveThermalSources,
      this);
  }
  
  /**
  @brief Default destructor
  @return void
  **/ 
  ThermalSensor::~ThermalSensor(void)
  {
    
  }

  /**
  @brief Updates the sensor measurements
  @return void
  **/ 
  void ThermalSensor::updateSensorCallback() 
  {
    if (thermal_sources_.thermal_sources.size() == 0) return;    

    stdr_msgs::ThermalSensorMeasurementMsg measuredSourcesMsg;

    measuredSourcesMsg.header.frame_id = _description.frame_id;

    
    float max_range = _description.maxRange;
    float sensor_th = tf::getYaw(_sensorTransform.getRotation());
    float min_angle = sensor_th - _description.angleSpan / 2.0;
    float max_angle = sensor_th + _description.angleSpan / 2.0;
    
    measuredSourcesMsg.thermal_source_degrees.push_back(0);
    //!< Must implement the functionality
    for(unsigned int i = 0 ; i < thermal_sources_.thermal_sources.size() ; i++)
    {
      //!< Check for max distance
      float sensor_x = _sensorTransform.getOrigin().x();
      float sensor_y = _sensorTransform.getOrigin().y();
      float dist = sqrt(
        pow(sensor_x - thermal_sources_.thermal_sources[i].pose.x, 2) +
        pow(sensor_y - thermal_sources_.thermal_sources[i].pose.y, 2)
      );
      if(dist > max_range)
      {
        continue;
      }
      
      //!< Check for correct angle
      float ang = atan2( 
        thermal_sources_.thermal_sources[i].pose.y - sensor_y,
        thermal_sources_.thermal_sources[i].pose.x - sensor_x);
      
      if(!stdr_robot::angCheck(ang, min_angle, max_angle))
      {
        continue;
      }
      
      // Returns the larger temperature found in its range
      if( thermal_sources_.thermal_sources[i].degrees >
        measuredSourcesMsg.thermal_source_degrees[0])
      {
        measuredSourcesMsg.thermal_source_degrees[0] = 
          thermal_sources_.thermal_sources[i].degrees;
      }
    }
    
    measuredSourcesMsg.header.stamp = ros::Time::now();
    measuredSourcesMsg.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish( measuredSourcesMsg );
  }
  
  /**
  @brief Receives the existent sources
  **/
  void ThermalSensor::receiveThermalSources(
    const stdr_msgs::ThermalSourceVector& msg)
  {
    thermal_sources_ = msg;
  }

}  // namespace stdr_robot
