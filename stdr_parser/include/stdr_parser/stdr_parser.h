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
      static Node* base_node_;
      
      /**
      @brief Parses an xml file
      @param file_name [std::string] The xml filename
      @return void
      **/
      static void parse(std::string file_name);
      
      /**
      @brief Recursive function - Expands the 'filename' nodes and eliminates them
      @param n [Node*] The stdr xml tree node to begin
      @return bool : True is a 'filename' node was expanded
      **/
      static bool eliminateFilenames(Node* n);
      
      /**
      @brief Recursive function - Merges the nodes that do not exist in non_mergable_tags_
      @param n [Node*] The stdr xml tree node to begin
      @return bool : True is a ndoe was merged
      **/
      static bool mergeNodes(Node* n);
      
      /**
      @brief Merges the leaves of the xml tree, which are the value nodes
      @param n [Node*] The stdr xml tree node to begin
      @return void
      **/
      static void mergeNodesValues(Node* n);
      
      /**
      @brief Default constructor
      @return void
      **/
      Parser(void);

    public:
      
      /**
      @brief Creates a message from a file
      @param file_name [std::string] The filename
      @return T : The message
      **/
      template <class T>
      static T createMessage(std::string file_name)
      {
        try
        {
          parse(file_name);
        }
        catch(ParserException ex)
        {
          delete base_node_;
          throw ex;
        }
        T msg = MessageCreator::createMessage<T>(base_node_,0);
        delete base_node_;
        return msg;
      }
      
      /**
      @brief Saves a stdr_msgs::Noise message to a file
      @param msg [T] The message
      @param file_name [std::string] The filename
      @return void
      **/
      template <class T>
      static void saveMessage(T msg,std::string file_name)
      {
        if(file_name.find(".xml") != std::string::npos)
        {
          XmlFileWriter::messageToFile(msg,file_name);  
        }
        else if(file_name.find(".yaml") != std::string::npos)
        {
          YamlFileWriter::messageToFile(msg,file_name); 
        }
      }

  };
}
#endif
