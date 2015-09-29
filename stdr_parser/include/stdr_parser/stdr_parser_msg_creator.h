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

#ifndef STDR_PARSER_MSG_CREATOR
#define STDR_PARSER_MSG_CREATOR

#include "stdr_parser/stdr_parser_node.h"
#include <sstream>

/**
@namespace stdr_parser
@brief The main namespace for STDR parser
**/ 
namespace stdr_parser
{
  
  /**
  @class MessageCreator
  @brief Creates STDR messages from a STDR tree
  **/ 
  class MessageCreator
  {
    private:
     
      /**
      @brief Default constructor
      @return void
      **/
      MessageCreator(void);
      
    public:
      
      /**
      @brief Creates a pose message from a parsed file
      @param n [Node*] The root node
      @return geometry_msgs::Pose2D
      **/
      template <typename T>
      static T createMessage(Node *n,unsigned int id);

      template <typename T>
      static T stringToType(std::string s)
      {
        std::stringstream str;
        str << s;
        T temp;
        str >> temp;
        return temp;
      }
  };
}
#endif
