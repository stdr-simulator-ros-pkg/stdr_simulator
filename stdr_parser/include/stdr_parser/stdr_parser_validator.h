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

#ifndef STDR_PARSER_VALIDATOR
#define STDR_PARSER_VALIDATOR

#include "stdr_parser/stdr_parser_node.h"
#include "stdr_parser/stdr_parser_tools.h"

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
  class Validator
  {
    private:

      //static ElSpecs elspecs_struct;
      static Specs specs_struct;
      /**
      @brief Low-level recursive function for parsing the xml specifications file
      @param node [TiXmlNode*] The xml node to start from
      @return void
      **/
      static std::map<std::string,ElSpecs> parseSpecifications(TiXmlNode* node);
      
      /**
      @brief Performs a allowed - validity check on the xml tree
      @param file_name [std::string] The filename from which the node came from
      @param n [Node*] The stdr xml tree node to begin
      @return void
      **/
      static void validityAllowedCheck(std::string file_name, Node* n);
      
      /**
      @brief Performs a required - validity check on the xml tree
      @param file_name [std::string] The filename from which the node came from
      @param n [Node*] The stdr xml tree node to begin
      @return void
      **/
      static void validityRequiredCheck(std::string file_name, Node* n);
      
      /**
      @brief Default constructor
      @return void
      **/
      Validator(void);
      
    public:

      /**
      @brief Performs a required / allowed - validity check on the xml tree
      @param file_name [std::string] The filename from which the tree was created
      @param n [Node*] The stdr xml tree node to begin
      @return void
      **/
      static void validate(std::string file_name, Node* n);
      
      /**
      @brief Parses the mergable specifications file
      @param file [std::string] The filename that contains the mergable specifications, including its path
      @return std::set<std::string> The non mergable tags
      **/
      static std::set<std::string> parseMergableSpecifications(std::string file);

      /**
      @brief Returns the non mergable tags
      @param void
      @return std::set<std::string> The non mergable tags
      **/
      static std::set<std::string> getNonMergableTags(void);

      /**
      @brief Returns the specifications for a specific tag
      @param tag [std::string] The tag 
      @return ElSpecs The structure that defines the required tags, allowed tags and default_value for the input tag
      **/
      static ElSpecs getSpecs(std::string tag);

      /**
      @brief Clears the static structure Specs
      @param void 
      @return Specs The empty structure
      **/
      static Specs clearSpecs(void);

   
      

    private:
      friend class ValidatorTest;
  };
}
#endif
