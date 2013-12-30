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
      
      stdr_msgs::RobotMsg createRobotMessage(Node *n);
      stdr_msgs::LaserSensorMsg createLaserMessage(Node *n,unsigned int id);
      stdr_msgs::SonarSensorMsg createSonarMessage(Node *n,unsigned int id);
      stdr_msgs::FootprintMsg createFootprintMessage(Node *n);
      stdr_msgs::Noise createNoiseMessage(Node *n);
      
      geometry_msgs::Pose2D createPoseMessage(Node *n);
  };
}
#endif
