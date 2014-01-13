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

#ifndef STDR_XML_PARSER__MSG_CREATOR
#define STDR_XML_PARSER__MSG_CREATOR

#include "stdr_xml_parser/stdr_xml_parser_node.h"

/**
@namespace stdr_xml_parser
@brief The main namespace for STDR GUI XML parser
**/ 
namespace stdr_xml_parser
{
  /**
  @class Base
  @brief Implements the main functionalities of the high-level parser
  **/ 
  class MessageCreator
  {
    private:
     
    public:
    
      /**
      @brief Default constructor
      @return void
      **/
      MessageCreator(void);
      
      /**
      @brief Creates a robot message from a parsed xml
      @param n [Node*] The root node
      @return stdr_msgs::RobotMsg
      **/
      stdr_msgs::RobotMsg createRobotMessage(Node *n);
      
      /**
      @brief Creates a laser message from a parsed xml
      @param n [Node*] The root node
      @param id [unsigned int] The id to create the laser frame_id 
      @return stdr_msgs::LaserSensorMsg
      **/
      stdr_msgs::LaserSensorMsg createLaserMessage(Node *n,unsigned int id);
      
      /**
      @brief Creates a sonar message from a parsed xml
      @param n [Node*] The root node
      @param id [unsigned int] The id to create the sonar frame_id 
      @return stdr_msgs::SonarSensorMsg
      **/
      stdr_msgs::SonarSensorMsg createSonarMessage(Node *n,unsigned int id);
      
      /**
      @brief Creates a footprint message from a parsed xml
      @param n [Node*] The root node
      @return stdr_msgs::FootprintMsg
      **/
      stdr_msgs::FootprintMsg createFootprintMessage(Node *n);
      
      /**
      @brief Creates a noise message from a parsed xml
      @param n [Node*] The root node
      @return stdr_msgs::Noise
      **/
      stdr_msgs::Noise createNoiseMessage(Node *n);
      
      /**
      @brief Creates a pose message from a parsed xml
      @param n [Node*] The root node
      @return geometry_msgs::Pose2D
      **/
      geometry_msgs::Pose2D createPoseMessage(Node *n);
      
      //----------------------------------------------------------------------//
      /**
      @brief Creates an xml from a noise msg
      @param msg [stdr_msgs::Noise] The noise message
      @param file_name [std::string] The file name to write the xml
      @return TiXmlDocument
      **/
      TiXmlDocument noiseToXml(stdr_msgs::Noise msg,std::string file_name);
  };
}
#endif
