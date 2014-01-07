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

#include <stdr_robot/sensors/sonar.h>

namespace stdr_robot {

  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::SonarSensorMsg&] The sonar description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/ 
  Sonar::Sonar(const nav_msgs::OccupancyGrid& map,
      const stdr_msgs::SonarSensorMsg& msg, 
      const std::string& name,
      ros::NodeHandle& n)
    : Sensor(map, name)
  {
    _description = msg;

    _timer = n.createTimer(
      ros::Duration(1 / msg.frequency), &Sonar::updateSensorCallback, this);  
    _tfTimer = n.createTimer(
      ros::Duration(1 / (2 * msg.frequency)), &Sonar::updateTransform, this);

    _publisher = n.advertise<sensor_msgs::Range>
      ( _namespace + "/" + msg.frame_id, 1 );
  }
  
  /**
  @brief Default destructor
  @return void
  **/ 
  Sonar::~Sonar(void)
  {
    
  }

  /**
  @brief Updates the sensor measurements
  @param ev [const ros::TimerEvent&] A ROS timer event
  @return void
  **/ 
  void Sonar::updateSensorCallback(const ros::TimerEvent&) 
  {
    if (!_gotTransform) { //!< wait for transform 
      return;
    }
    
    float angle;
    int distance;
    int xMap, yMap;
    sensor_msgs::Range sonarRangeMsg;

    sonarRangeMsg.max_range = _description.maxRange;
    sonarRangeMsg.min_range = _description.minRange;
    sonarRangeMsg.radiation_type = 0;
    sonarRangeMsg.field_of_view = _description.coneAngle;

    if ( _map.info.height == 0 || _map.info.width == 0 )
    {
      ROS_DEBUG("Outside limits\n");
      return;
    }

    sonarRangeMsg.range = _description.maxRange;

    float angleStep = 3.14159 / 180.0;
    float angleMin = - ( _description.coneAngle / 2.0 ); 
    float angleMax = _description.coneAngle / 2.0 ; 
    ////float angleStep = _description.coneAngle * 180.0 / 3.14159;
    //float angleMin = -( _description.coneAngle * 180.0 / 3.14159 / 2.0 ); 
    //float angleMax = _description.coneAngle * 180.0 / 3.14159 / 2.0 ; 

    for ( float sonarIter = angleMin; sonarIter < angleMax; 
      sonarIter += angleStep )
    {

      distance = 1;

      while ( distance <= _description.maxRange / _map.info.resolution )
      {
        xMap = _sensorTransform.getOrigin().x() / _map.info.resolution + 
          cos( sonarIter + tf::getYaw(_sensorTransform.getRotation()) ) 
            * distance;
        yMap = _sensorTransform.getOrigin().y() / _map.info.resolution + 
          sin( sonarIter + tf::getYaw(_sensorTransform.getRotation()) ) 
            * distance;
        
        //!< Found obstacle
        if ( _map.data[ yMap * _map.info.width + xMap ] > 70 )
        {
          break;
        }
        distance ++;
      }

      if ( distance * _map.info.resolution < sonarRangeMsg.range )
      {
        sonarRangeMsg.range = distance * _map.info.resolution;
      }
    }
    
    if ( sonarRangeMsg.range < _description.minRange )
    {
      sonarRangeMsg.range = -std::numeric_limits<float>::infinity();
    }
    else if ( sonarRangeMsg.range >= _description.maxRange )
    {
      sonarRangeMsg.range = std::numeric_limits<float>::infinity();
    }

    sonarRangeMsg.header.stamp = ros::Time::now();
    sonarRangeMsg.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish( sonarRangeMsg );
  }

  /**
  @brief Returns the sensor pose relatively to robot
  @return geometry_msgs::Pose2D
  **/ 
  geometry_msgs::Pose2D Sonar::getSensorPose()
  {
    return _description.pose;
  }
  
  /**
  @brief Returns the sensor frame id
  @return std::string
  **/ 
  std::string Sonar::getFrameId()
  {
    return _namespace + "_" + _description.frame_id;
  }
  
  /**
  @brief Updates the sensor tf transform
  @param ev [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void Sonar::updateTransform(const ros::TimerEvent&)
  {
    try 
    {
      _tfListener.waitForTransform(
        "map_static",
        _namespace + "_" + _description.frame_id,
        ros::Time(0),
        ros::Duration(1.0));
        
      _tfListener.lookupTransform(
        "map_static",
        _namespace + "_" + _description.frame_id,
        ros::Time(0), 
        _sensorTransform);
      
      _gotTransform = true;
    }
    catch (tf::TransformException ex) 
    {
      ROS_WARN("%s", ex.what());
    }
    
  }

}
