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

#include <stdr_server/map_server.h>

namespace stdr_server {

  /**
  @brief Constructor by filename
  @param fname [const std::string&] The file name
  @return void
  **/
  MapServer::MapServer(const std::string& fname)
  {

    map_ = map_loader::loadMap(fname);

    meta_data_message_ = map_.info;

    publishData();
  }

  /**
  @brief Constructor by occupancy grid map
  @param map [const nav_msgs::OccupancyGrid&] The occupancy grid map
  @return void
  **/
  MapServer::MapServer(const nav_msgs::OccupancyGrid& map)
  {

    map_ = map;

    meta_data_message_ = map_.info;

    publishData();
  }

  /**
  @brief Publishes the map data and metadata
  @return void
  **/
  void MapServer::publishData(void)
  {

    tfTimer = n.createTimer(ros::Duration(0.1),
      &MapServer::publishTransform, this);

    //!< Latched publisher for metadata
    metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    metadata_pub.publish( meta_data_message_ );

    //!< Latched publisher for data
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_pub.publish( map_ );
  }

  /**
  @brief Publishes the map to map_static transform
  @param ev [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void MapServer::publishTransform(const ros::TimerEvent&) {

    tf::Vector3 translation(
      map_.info.origin.position.x,
      map_.info.origin.position.y,
      0);

    tf::Quaternion rotation;

    rotation.setRPY(0, 0, tf::getYaw(map_.info.origin.orientation));

    tf::Transform worldTomap(rotation, translation);

    tfBroadcaster.sendTransform(
      tf::StampedTransform(worldTomap, ros::Time::now(), "map", "map_static"));

  }

} // end of namespace stdr_server

