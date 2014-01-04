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

#ifndef STDR_XML_PARSER__TOOLS
#define STDR_XML_PARSER__TOOLS

#include <iostream>
#include <cstdlib>
#include <map>
#include <vector>
#include <string>
#include <sstream>

#include <ros/package.h>
#include "ros/ros.h"

#include <tinyxml.h>

#include "stdr_msgs/RobotMsg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"

//!< Transforms a float number to string
#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

/**
@namespace stdr_xml_parser
@brief The main namespace for STDR GUI XML parser
**/ 
namespace stdr_xml_parser
{
  /**
  @brief Explodes a string based on a delimiter
  @param s [std::string] The input string
  @param delimiter [char] The delimiter
  @return std::set<std::string> : An ensemble of strings
  **/
  std::set<std::string> explodeString(std::string s,char delimiter);
}
#endif
