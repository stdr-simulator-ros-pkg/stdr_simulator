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

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/ 
namespace stdr_robot {
  /**
  @namespace stdr_parser
  @brief The main namespace for STDR YAML parser
  **/ 
  namespace parser {
 
    /**
    @brief Reads a yaml file and produces a robot message
    @param filename [const std::string&] File name
    @return stdr_msgs::RobotMsg
    **/
    stdr_msgs::RobotMsg yamlToRobotMsg(const std::string& filename);
    
    /**
    @brief Reads a yaml file and produces a laser sensor message
    @param filename [const std::string&] File name
    @return stdr_msgs::LaserSensorMsg
    **/
    stdr_msgs::LaserSensorMsg yamlToLaserSensorMsg
      (const std::string& filename);
    
    /**
    @brief Reads a yaml file and produces a sonar sensor message
    @param filename [const std::string&] File name
    @return stdr_msgs::SonarSensorMsg
    **/
    stdr_msgs::SonarSensorMsg yamlToSonarSensorMsg
      (const std::string& filename);
    
    /**
    @brief Reads a yaml file and produces an rfid antenna sensor message
    @param filename [const std::string&] File name
    @return stdr_msgs::RfidSensorMsg
    **/
    stdr_msgs::RfidSensorMsg yamlToRfidSensorMsg(const std::string& filename);
    
    // maybe use template
    /**
    @brief Writes a robot message to a yaml file 
    @param filename [const std::string& filename] The file name
    @param msg [stdr_msgs::RobotMsg&] The message
    @return void
    **/
    void robotMsgToYaml(const std::string& filename, 
      const stdr_msgs::RobotMsg& msg);
    
    /**
    @brief Writes a laser sensor message to a yaml file 
    @param filename [const std::string& filename] The file name
    @param msg [stdr_msgs::LaserSensorMsg&] The message
    @return void
    **/
    void laserSensorMsgToYaml(const std::string& filename, 
      const stdr_msgs::LaserSensorMsg& msg);
      
    /**
    @brief Writes a sonar sensor message to a yaml file 
    @param filename [const std::string& filename] The file name
    @param msg [stdr_msgs::SonarSensorMsg&] The message
    @return void
    **/
    void sonarSensorMsgToYaml(const std::string& filename, 
      const stdr_msgs::SonarSensorMsg& msg);
      
    /**
    @brief Writes an rfid antenna sensor message to a yaml file 
    @param filename [const std::string& filename] The file name
    @param msg [stdr_msgs::RfidSensorMsg&] The message
    @return void
    **/
    void rfidSensorMsgToYaml(const std::string& filename, 
      const stdr_msgs::RfidSensorMsg& msg);
    
    // operators for parsing
    /**
    @brief Fills a robot message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::RobotMsg&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::RobotMsg& msg);
    
    /**
    @brief Fills a pose 2D message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [geometry_msgs::Pose2D&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, geometry_msgs::Pose2D& msg);
    
    /**
    @brief Fills a noise message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::Noise&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::Noise& msg);
    
    /**
    @brief Fills a Point message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [geometry_msgs::Point&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, geometry_msgs::Point& msg);
    
    /**
    @brief Fills a laser sensor message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::LaserSensorMsg&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::LaserSensorMsg& msg);
    
    /**
    @brief Fills a sonar sensor message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::SonarSensorMsg&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::SonarSensorMsg& msg);
    
    /**
    @brief Fills an rfid antenna sensor message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::SonarSensorMsg&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::RfidSensorMsg& msg);
    
    /**
    @brief Fills a footprint message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::FootprintMsg&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::FootprintMsg& msg);
    
    /**
    @brief Fills a kinematic message from a YAML node
    @param node [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::KinematicMsg&] The message
    @return void
    **/
    void operator >> (const YAML::Node& node, stdr_msgs::KinematicMsg& msg);
    
    // operators for emitting
    /**
    @brief Fills a YAML node from a robot message
    @param out [const YAML::Node&] The yaml node
    @param msg [stdr_msgs::RobotMsg&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const stdr_msgs::RobotMsg& msg);
      
    /**
    @brief Fills a YAML node from a pose message
    @param out [const YAML::Node&] The yaml node
    @param msg [const geometry_msgs::Pose2D&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const geometry_msgs::Pose2D& msg);
      
    /**
    @brief Fills a YAML node from a laser sensor message
    @param out [const YAML::Node&] The yaml node
    @param msg [const stdr_msgs::LaserSensorMsg&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const stdr_msgs::LaserSensorMsg& msg);
      
    /**
    @brief Fills a YAML node from a sonar sensor message
    @param out [const YAML::Node&] The yaml node
    @param msg [const stdr_msgs::SonarSensorMsg&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const stdr_msgs::SonarSensorMsg& msg);
      
    /**
    @brief Fills a YAML node from an rfid antenna sensor message
    @param out [const YAML::Node&] The yaml node
    @param msg [const stdr_msgs::RfidSensorMsg&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const stdr_msgs::RfidSensorMsg& msg);
      
    /**
    @brief Fills a YAML node from a footprint message
    @param out [const YAML::Node&] The yaml node
    @param msg [const stdr_msgs::FootprintMsg&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const stdr_msgs::FootprintMsg& msg);
      
    /**
    @brief Fills a YAML node from a point message
    @param out [const YAML::Node&] The yaml node
    @param msg [const geometry_msgs::Point&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const geometry_msgs::Point& msg);
      
    /**
    @brief Fills a YAML node from a noise message
    @param out [const YAML::Node&] The yaml node
    @param msg [const stdr_msgs::Noise&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out,  
      const stdr_msgs::Noise& msg);
      
    /**
    @brief Fills a YAML node from a kinematic message
    @param out [const YAML::Node&] The yaml node
    @param msg [const stdr_msgs::KinematicMsg&] The message
    @return void
    **/
    YAML::Emitter& operator << (YAML::Emitter& out, 
      const stdr_msgs::KinematicMsg& msg);

  } // end of namespace parser 
    
} // end of namespace stdr_robot 

#endif
