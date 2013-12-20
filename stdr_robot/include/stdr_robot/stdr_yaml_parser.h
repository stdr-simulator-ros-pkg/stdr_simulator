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

#ifndef STDR_YAML_PARSER_H
#define STDR_YAML_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "yaml-cpp/yaml.h"

#include "stdr_msgs/RobotMsg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

namespace stdr_robot {
  
  namespace parser {
 
    stdr_msgs::RobotMsg yamlToRobotMsg(const std::string& filename);
    stdr_msgs::LaserSensorMsg yamlToLaserSensorMsg(const std::string& filename);
    stdr_msgs::SonarSensorMsg yamlToSonarSensorMsg(const std::string& filename);
    stdr_msgs::RfidSensorMsg yamlToRfidSensorMsg(const std::string& filename);
    
    // maybe use template
    void robotMsgToYaml(const std::string& filename, const stdr_msgs::RobotMsg& msg);
    void laserSensorMsgToYaml(const std::string& filename, const stdr_msgs::LaserSensorMsg& msg);
    void sonarSensorMsgToYaml(const std::string& filename, const stdr_msgs::SonarSensorMsg& msg);
    void rfidSensorMsgToYaml(const std::string& filename, const stdr_msgs::RfidSensorMsg& msg);
    
    // operators for parsing
    void operator >> (const YAML::Node& node, stdr_msgs::RobotMsg& msg);
    void operator >> (const YAML::Node& node, geometry_msgs::Pose2D& msg);
    void operator >> (const YAML::Node& node, stdr_msgs::Noise& msg);
    void operator >> (const YAML::Node& node, geometry_msgs::Point& msg);
    void operator >> (const YAML::Node& node, stdr_msgs::LaserSensorMsg& msg);
    void operator >> (const YAML::Node& node, stdr_msgs::SonarSensorMsg& msg);
    void operator >> (const YAML::Node& node, stdr_msgs::RfidSensorMsg& msg);
    void operator >> (const YAML::Node& node, stdr_msgs::FootprintMsg& msg);
    void operator >> (const YAML::Node& node, stdr_msgs::KinematicMsg& msg);
    
    // operators for emitting
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::RobotMsg& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Pose2D& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::LaserSensorMsg& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::SonarSensorMsg& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::RfidSensorMsg& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::FootprintMsg& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Point& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::Noise& msg);
    YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::KinematicMsg& msg);

  } // end of namespace parser 
    
} // end of namespace stdr_robot 

#endif
