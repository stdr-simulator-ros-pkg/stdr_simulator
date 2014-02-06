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

#ifndef STDR_PARSER_YAML
#define STDR_PARSER_YAML

#include "stdr_parser/stdr_parser_node.h"
#include "stdr_parser/stdr_parser_exceptions.h"

/**
@namespace stdr_parser
@brief The main namespace for STDR GUI XML parser
**/ 
namespace stdr_parser
{
  /**
  @class Parser
  @brief Implements the main functionalities of the high-level parser
  **/ 
  class YamlParser
  {
    private:
     
      /**
      @brief Default constructor
      @return void
      **/
      YamlParser(void);
     
      /**
      @brief Low-level recursive function for parsing the yaml file
      @param node [YAML::Node&] The yaml node to start from
      @param n [Node*] The stdr tree node to update
      @return void
      **/
      static void parseLow(const YAML::Node& node,Node* n);
      
    public:
    
      
      
      /**
      @brief Private function that initiates the parsing of an xml file
      @param file_name [std::string] The xml file name
      @param n [Node*] The stdr xml tree node to update
      @return void
      **/
      static void parse(std::string file_name,Node* n);
  };
}
#endif
