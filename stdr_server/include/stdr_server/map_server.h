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

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "stdr_server/map_loader.h"

/**
@namespace stdr_server
@brief The main namespace for STDR Server
**/
namespace stdr_server {

  /**
  @class MapServer
  @brief Implements the STDR map server functionalities
  **/
  class MapServer
  {
    public:

      /**
      @brief Constructor by filename
      @param fname [const std::string&] The file name
      @return void
      **/
      explicit MapServer(const std::string& fname);

      /**
      @brief Constructor by occupancy grid map
      @param map [const nav_msgs::OccupancyGrid&] The occupancy grid map
      @return void
      **/
      explicit MapServer(const nav_msgs::OccupancyGrid& map);

    private:

      /**
      @brief Publishes the map data and metadata
      @return void
      **/
      void publishData();

      /**
      @brief Publishes the map to map_static transform
      @param ev [const ros::TimerEvent&] A ROS timer event
      @return void
      **/
      void publishTransform(const ros::TimerEvent& ev);

    private:

      //!< The ROS node handle
      ros::NodeHandle n;
      //!< ROS publisher for posting the map
      ros::Publisher map_pub;
      //!< ROS publisher for posting the map metadata
      ros::Publisher metadata_pub;
      //!< ROS timer for tf posting
      ros::Timer tfTimer;
      //!< ROS tf broadcaster
      tf::TransformBroadcaster tfBroadcaster;
      //!< ROS map metadata message
      nav_msgs::MapMetaData meta_data_message_;
      //!< ROS occupancy grid message
      nav_msgs::OccupancyGrid map_;

  };
}  // namespace stdr_server
