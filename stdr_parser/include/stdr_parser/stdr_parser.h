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

#ifndef STDR_PARSER_BASE
#define STDR_PARSER_BASE

#include "stdr_parser/stdr_parser_msg_creator.h"
#include "stdr_parser/stdr_parser_validator.h"
#include "stdr_parser/stdr_parser_xml_file_writer.h"
#include "stdr_parser/stdr_parser_yaml_file_writer.h"
#include "stdr_parser/stdr_xml_parser.h"
#include "stdr_parser/stdr_yaml_parser.h"

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
  class Parser
  {
    private:
      
      //!< Base node of the parsed file
      Node* base_node_;
      //!< The path for stdr_resources
      std::string base_path_;
      
      
      //!< Object of Message creator
      MessageCreator creator_;
      //!< Node tree validator
      Validator validator_;
      //!< Extracts messages in xml files
      XmlFileWriter xml_file_writer_;
      //!< Extracts messages in yaml files
      YamlFileWriter yaml_file_writer_;
      //!< Parses an xml file
      XmlParser xml_parser_;
      //!< Parses a yaml file
      YamlParser yaml_parser_;
      
      /**
      @brief Parses an xml file
      @param file_name [std::string] The xml filename
      @return void
      **/
      void parse(std::string file_name);
      
      /**
      @brief Recursive function - Expands the 'filename' nodes and eliminates them
      @param n [Node*] The stdr xml tree node to begin
      @return bool : True is a 'filename' node was expanded
      **/
      bool eliminateFilenames(Node* n);
      
      /**
      @brief Recursive function - Merges the nodes that do not exist in non_mergable_tags_
      @param n [Node*] The stdr xml tree node to begin
      @return bool : True is a ndoe was merged
      **/
      bool mergeNodes(Node* n);
      
      /**
      @brief Merges the leaves of the xml tree, which are the value nodes
      @param n [Node*] The stdr xml tree node to begin
      @return void
      **/
      void mergeNodesValues(Node* n);

    public:
    
      /**
      @brief Default constructor
      @return void
      **/
      Parser(void);
      
      /**
      @brief Parses a file and produces a stdr_msgs::RobotMsg message
      @param file_name [std::string] The filename
      @return stdr_msgs::RobotMsg : The robot message
      **/
      stdr_msgs::RobotMsg createRobotMessage(std::string file_name);
      
      /**
      @brief Parses a file and produces a stdr_msgs::LaserSensorMsg message
      @param file_name [std::string] The filename
      @return stdr_msgs::LaserSensorMsg : The laser message
      **/
      stdr_msgs::LaserSensorMsg createLaserMessage(std::string file_name);
      
      /**
      @brief Parses a file and produces a stdr_msgs::SonarSensorMsg message
      @param file_name [std::string] The filename
      @return stdr_msgs::SonarSensorMsg : The sonar message
      **/
      stdr_msgs::SonarSensorMsg createSonarMessage(std::string file_name);
      
      /**
      @brief Parses a file and produces a stdr_msgs::Noise message
      @param file_name [std::string] The filename
      @return stdr_msgs::Noise : The noise message
      **/
      stdr_msgs::Noise createNoiseMessage(std::string file_name);
      
      /**
      @brief Parses a file and produces a stdr_msgs::FootprintMsg message
      @param file_name [std::string] The filename
      @return stdr_msgs::FootprintMsg : The footprint message
      **/
      stdr_msgs::FootprintMsg createFootprintMessage(std::string file_name);
      
      //---------------------------------------------------------------------------//
      
      /**
      @brief Saves a stdr_msgs::Noise message to a yaml or xml file
      @param msg [stdr_msgs::Noise] The noise message
      @param file_name [std::string] The filename
      @return void
      **/
      void saveNoiseMessage(stdr_msgs::Noise msg,std::string file_name);
      
      /**
      @brief Saves a stdr_msgs::FootprintMsg message to a file
      @param msg [stdr_msgs::FootprintMsg] The footprint message
      @param file_name [std::string] The filename
      @return void
      **/
      void saveFootprintMessage(
        stdr_msgs::FootprintMsg msg,std::string file_name);
  };
}
#endif
