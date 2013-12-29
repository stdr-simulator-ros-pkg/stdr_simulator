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

#ifndef STDR_XML_PARSER__H
#define STDR_XML_PARSER__H

#include <iostream>
#include <cstdlib>
#include <map>
#include <vector>
#include <string>

#include <ros/package.h>
#include "ros/ros.h"

#include <tinyxml.h>

#include "stdr_msgs/RobotMsg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"

namespace stdr_xml_parser
{
  struct ElSpecs
  {
    ElSpecs(void);
    std::set<std::string> required;
    std::set<std::string> allowed;
  };

  struct Specs
  {
    Specs(void);
    static std::map<std::string,ElSpecs> specs;
  };

  class Node
  {
    private:
      std::vector<Node> elements_;
      std::map<std::string,std::string> attributes_;
    public:
      void parse(std::string file_name);
  };

  class Base
  {
    private:
      Node base_node_;
      std::string base_path_;
    public:
      Base(void);
      void initializeParsing(std::string file_name);
      void loadSpecifications(void);
      void parseSpecifications(TiXmlNode* node);
  };
}

#endif
