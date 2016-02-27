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

#include <stdr_robot/sensors/laser.h>

namespace stdr_robot {

  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::LaserSensorMsg&] The laser description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/ 
  Laser::Laser(const nav_msgs::OccupancyGrid& map,
      const stdr_msgs::LaserSensorMsg& msg, 
      const std::string& name,
      ros::NodeHandle& n)
  : 
    Sensor(map, name, n, msg.pose, msg.frame_id, msg.frequency)
  {
    _description = msg;

    _publisher = n.advertise<sensor_msgs::LaserScan>
      ( _namespace + "/" + msg.frame_id, 1 );
  }

  /**
  @brief Updates the sensor measurements
  @return void
  **/ 
  void Laser::updateSensorCallback() 
  {
    float angle;
    int distance;
    int xMap, yMap;
    int divisions = 1;
    sensor_msgs::LaserScan _laserScan;

    if ( _description.numRays > 1 )
    {
        divisions = _description.numRays - 1;
    }
    
    _laserScan.angle_min = _description.minAngle;
    _laserScan.angle_max = _description.maxAngle;
    _laserScan.range_max = _description.maxRange;
    _laserScan.range_min = _description.minRange;
    _laserScan.angle_increment = 
      ( _description.maxAngle - _description.minAngle ) / divisions;


    if ( _map.info.height == 0 || _map.info.width == 0 ) 
    {
      ROS_DEBUG("Outside limits\n");
      return;
    }
    for ( int laserScanIter = 0; laserScanIter < _description.numRays; 
      laserScanIter++ )
    {

      angle = tf::getYaw(_sensorTransform.getRotation()) + 
        _description.minAngle + laserScanIter * 
          ( _description.maxAngle - _description.minAngle ) 
            / divisions;
      
      distance = 1;

      while ( distance <= _description.maxRange / _map.info.resolution )
      {
        xMap = _sensorTransform.getOrigin().x() / _map.info.resolution + 
          cos( angle ) * distance;
        
        yMap = _sensorTransform.getOrigin().y() / _map.info.resolution + 
          sin( angle ) * distance;
        
        if (yMap * _map.info.width + xMap > _map.info.height*_map.info.width ||
            yMap * _map.info.width + xMap < 0)
        {
          distance = _description.maxRange / _map.info.resolution - 1;
          break;
        }
        
        if ( _map.data[ yMap * _map.info.width + xMap ] > 70 )
        {
          break;
        }
        
        distance ++;
      }

      if ( distance * _map.info.resolution > _description.maxRange )
        _laserScan.ranges.push_back( std::numeric_limits<float>::infinity() );
      else if ( distance * _map.info.resolution < _description.minRange )
        _laserScan.ranges.push_back(- std::numeric_limits<float>::infinity() );
      else
        _laserScan.ranges.push_back( distance * _map.info.resolution );
    }
    
    _laserScan.header.stamp = ros::Time::now();
    _laserScan.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish( _laserScan );
  }

}  // namespace stdr_robot
