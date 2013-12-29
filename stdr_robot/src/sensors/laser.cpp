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

  Laser::Laser(const nav_msgs::OccupancyGrid& map,
      const stdr_msgs::LaserSensorMsg& msg, 
      const std::string& name,
      ros::NodeHandle& n)
    : Sensor(map, name)
  {
    _description = msg;

    _timer = n.createTimer(ros::Duration(1/msg.frequency), &Laser::updateSensorCallback, this);	
    _tfTimer = n.createTimer(ros::Duration(1/(2*msg.frequency)), &Laser::updateTransform, this);	

    _publisher = n.advertise<sensor_msgs::LaserScan>( _namespace+"/"+msg.frame_id, 1 );
  }

  void Laser::updateSensorCallback(const ros::TimerEvent&) {
    
    if (!_gotTransform) { // wait for transform 
      return;
    }
    
    float angle;
    int distance;
    int xMap, yMap;
    sensor_msgs::LaserScan _laserScan;

    _laserScan.angle_min = _description.minAngle;
    _laserScan.angle_max = _description.maxAngle;
    _laserScan.range_max = _description.maxRange;
    _laserScan.range_min = _description.minRange;
    _laserScan.angle_increment = ( _description.maxAngle - _description.minAngle ) / _description.numRays;


    if ( _map.info.height == 0 || _map.info.width == 0 ) 
    {
      ROS_DEBUG("Outside limits\n");
      return;
    }
    for ( int laserScanIter = 0; laserScanIter < _description.numRays; laserScanIter++ )
    {

      angle = tf::getYaw(_sensorTransform.getRotation()) + _description.minAngle + laserScanIter * ( _description.maxAngle - _description.minAngle ) / _description.numRays;
      distance = 1;

      while ( distance <= _description.maxRange / _map.info.resolution )
      {
        xMap = _sensorTransform.getOrigin().x() / _map.info.resolution + cos( angle ) * distance;
        
        yMap = _sensorTransform.getOrigin().y() / _map.info.resolution + sin( angle ) * distance;
        
        if ( _map.data[ yMap * _map.info.width + xMap ] > 70 ) break;
        distance ++;
      }

      if ( distance * _map.info.resolution > _description.maxRange )
        _laserScan.ranges.push_back( std::numeric_limits<float>::infinity() );
      else if ( distance * _map.info.resolution < _description.minRange )
        _laserScan.ranges.push_back( -std::numeric_limits<float>::infinity() );
      else
        _laserScan.ranges.push_back( distance * _map.info.resolution );
    }
    
    _laserScan.header.stamp = ros::Time::now();
    _laserScan.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish( _laserScan );
  }

  geometry_msgs::Pose2D Laser::getSensorPose() 
  {
    return _description.pose;
  }
  
  std::string Laser::getFrameId()
  {
    return _namespace + "_" + _description.frame_id;
  }
  
  void Laser::updateTransform(const ros::TimerEvent&)
  {
    try {
      _tfListener.waitForTransform("map_static",
                                  _namespace + "_" + _description.frame_id,
                                  ros::Time(0),
                                  ros::Duration(0.2));
      _tfListener.lookupTransform("map_static",
                                  _namespace + "_" + _description.frame_id,
                                  ros::Time(0), _sensorTransform);
      _gotTransform = true;
    }
    catch (tf::TransformException ex) {
      ROS_WARN("%s",ex.what());
    }
    
  }

}
