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

#include "stdr_server/map_loader.h"

namespace stdr_server {

  namespace map_loader {

    /**
    @brief Loads a map from an image file
    @param fname [const std::string&] The file name
    @return nav_msgs::OccupancyGrid
    **/
    nav_msgs::OccupancyGrid loadMap(const std::string& fname) {

      nav_msgs::GetMap::Response map_resp_;

      std::string mapfname = "";
      std::ifstream fin(fname.c_str());
      double origin[3];
      double res;
      int negate;
      double occ_th, free_th;
      std::string frame_id = "map";

    #ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
      try {
        doc["resolution"] >> res;
      } catch (YAML::InvalidScalar) {
        ROS_ERROR(
          "The map does not contain a resolution tag or it is invalid.");
        exit(-1);
      }
      try {
        doc["negate"] >> negate;
      } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        exit(-1);
      }
      try {
        doc["occupied_thresh"] >> occ_th;
      } catch (YAML::InvalidScalar) {
        ROS_ERROR(
          "The map does not contain an occupied_thresh tag or it is invalid.");
        exit(-1);
      }
      try {
        doc["free_thresh"] >> free_th;
      } catch (YAML::InvalidScalar) {
        ROS_ERROR(
          "The map does not contain a free_thresh tag or it is invalid.");
        exit(-1);
      }
      try {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
      } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        exit(-1);
      }
      try {
        doc["image"] >> mapfname;
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
        ROS_ERROR("The image tag cannot be an empty string.");
        exit(-1);
        }
        if(mapfname[0] != '/')
        {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
        }
      } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),
        res,negate,occ_th,free_th, origin);

      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map_resp_.map.info.width,
           map_resp_.map.info.height,
           map_resp_.map.info.resolution);

      return map_resp_.map;
    }

  } // end of namespace map_loader

} // end of namespace stdr_server
