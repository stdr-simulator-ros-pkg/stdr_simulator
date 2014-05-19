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

#include <stdr_robot/sensors/co2.h>

namespace stdr_robot {
  
  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::CO2SensorMsg&] The sensor description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/ 
  CO2Sensor::CO2Sensor(
    const nav_msgs::OccupancyGrid& map,
    const stdr_msgs::CO2SensorMsg& msg, 
    const std::string& name,
    ros::NodeHandle& n)
    : Sensor(map, name, n, msg.pose, msg.frame_id, msg.frequency)
  {
    _description = msg;

    _publisher = n.advertise<stdr_msgs::CO2SensorMeasurementMsg>
      ( _namespace + "/" + msg.frame_id, 1 );
      
    co2_sources_subscriber_ = n.subscribe(
      "stdr_server/co2_sources_list", 
      1, 
      &CO2Sensor::receiveCO2Sources,
      this);
  }
  
  /**
  @brief Default destructor
  @return void
  **/ 
  CO2Sensor::~CO2Sensor(void)
  {
    
  }

  /**
  @brief Updates the sensor measurements
  @return void
  **/ 
  void CO2Sensor::updateSensorCallback() 
  {
    if (co2_sources_.co2_sources.size() == 0) return;    

    stdr_msgs::CO2SensorMeasurementMsg measuredSourcesMsg;

    measuredSourcesMsg.header.frame_id = _description.frame_id;

    float max_range = _description.maxRange;
    ///!< Must implement the functionality
    for(unsigned int i = 0 ; i < co2_sources_.co2_sources.size() ; i++)
    {
      //!< Calculate distance
      float sensor_x = _sensorTransform.getOrigin().x();
      float sensor_y = _sensorTransform.getOrigin().y();
      float dist = sqrt(
        pow(sensor_x - co2_sources_.co2_sources[i].pose.x, 2) +
        pow(sensor_y - co2_sources_.co2_sources[i].pose.y, 2)
      );
      if(dist > max_range)
      {
        continue;
      }
      if(dist > 0.5)
      {
        measuredSourcesMsg.co2_ppm += co2_sources_.co2_sources[i].ppm *
          pow(0.5, 2) / pow(dist, 2);
      }
      else
      {
        measuredSourcesMsg.co2_ppm += co2_sources_.co2_sources[i].ppm;
      }
    }
    
    measuredSourcesMsg.header.stamp = ros::Time::now();
    measuredSourcesMsg.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish( measuredSourcesMsg );
  }
  
  /**
  @brief Receives the existent sources
  **/
  void CO2Sensor::receiveCO2Sources(const stdr_msgs::CO2SourceVector& msg)
  {
    co2_sources_ = msg;
  }

}  // namespace stdr_robot
