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
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "stdr_server/map_loader.h"

namespace stdr_server {

class MapServer
{
  public:
    
    explicit MapServer(const std::string& fname);
    explicit MapServer(const nav_msgs::OccupancyGrid& map);
    
  private:
    
    void publishData();
  
  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::OccupancyGrid map_;

};
}
