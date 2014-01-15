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

#include "stdr_parser/stdr_parser.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  Parser::Parser(void)
  {
    base_path_=ros::package::getPath("stdr_resources");
    base_node_ = new Node();
    base_node_->tag = "STDR_Parser_Root_Node";
  }

  /**
  @brief Parses an xml file
  @param file_name [std::string] The xml filename
  @return void
  **/
  void Parser::parse(std::string file_name)
  {
    // Must destroy prev tree
    try
    {
      if(file_name.find(".xml") != std::string::npos)
      {
        xml_parser_.parse(file_name,base_node_);  
      }
      else if(file_name.find(".yaml") != std::string::npos)
      {
        yaml_parser_.parse(file_name,base_node_);
      }
      validator_.validate(base_node_);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    catch(YAML::ParserException& e)
    {
      std::string error = 
        std::string("STDR parser : Failed to load file '") + 
        file_name + std::string("'") +
        std::string("\nError was '") + std::string(e.what());
      throw ParserException(error);
    }
  }
 
  /**
  @brief Parses an xml file and produces a stdr_msgs::RobotMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::RobotMsg : The robot message
  **/
  stdr_msgs::RobotMsg Parser::createRobotMessage(std::string file_name)
  {
    
    try
    {
      parse(file_name);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    return creator_.createRobotMessage(base_node_);
  }
  
  /**
  @brief Parses an xml file and produces a stdr_msgs::LaserSensorMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::LaserSensorMsg : The laser message
  **/
  stdr_msgs::LaserSensorMsg Parser::createLaserMessage(std::string file_name)
  {
    
    try
    {
      parse(file_name);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    
    return creator_.createLaserMessage(base_node_,0);
  }
  
  /**
  @brief Parses an xml file and produces a stdr_msgs::SonarSensorMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::SonarSensorMsg : The sonar message
  **/
  stdr_msgs::SonarSensorMsg Parser::createSonarMessage(std::string file_name)
  {
    
    try
    {
      parse(file_name);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    return creator_.createSonarMessage(base_node_,0);
  }
}

