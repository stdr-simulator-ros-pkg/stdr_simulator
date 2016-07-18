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

#ifndef STDR_MAP_LOADER_H
#define STDR_MAP_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <string>
#include "ros/ros.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"


#ifdef HAVE_NEW_YAMLCPP
//! The >> operator disappeared in yaml-cpp 0.5, so this function is
//! added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

/**
@namespace stdr_server
@brief The main namespace for STDR Server
**/
namespace stdr_server {

  /**
  @namespace map_loader
  @brief The namespace for STDR map loader
  **/
  namespace map_loader {

    /**
    @brief Loads a map from an image file
    @param fname [const std::string&] The file name
    @return nav_msgs::OccupancyGrid
    **/
    nav_msgs::OccupancyGrid loadMap(const std::string& fname);

  } // end of namespace map_loader

} // end of namespace stdr_server


#endif
