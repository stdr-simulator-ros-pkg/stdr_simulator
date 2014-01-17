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

#include "stdr_parser/stdr_yaml_parser.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  YamlParser::YamlParser(void)
  {

  }
  
  /**
  @brief Parses an xml file
  @param file_name [std::string] The xml filename
  @return void
  **/
  void YamlParser::parse(std::string file_name, Node* base_node)
  {
    std::string path = file_name;
    std::ifstream fin(path.c_str());
    #ifdef HAVE_NEW_YAMLCPP
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
    
    base_node->file_origin = file_name;
    base_node->file_row = doc.GetMark().line;
    
    parseLow(doc,base_node);
  }
  
  /**
  @brief Low-level recursive function for parsing the yaml file
  @param node [YAML::Node&] The yaml node to start from
  @param n [Node*] The stdr tree node to update
  @return void
  **/
  void YamlParser::parseLow(const YAML::Node& node,Node* n)
  {
    if(node.Type() == YAML::NodeType::Scalar)
    {
      Node* new_node = new Node();
      std::string s;
      node >> s;
      new_node->value = s;
      new_node->file_origin = n->file_origin;
      new_node->file_row = node.GetMark().line;
      n->elements.push_back(new_node);
    }
    else if(node.Type() == YAML::NodeType::Sequence)
    {
      for(unsigned int i = 0 ; i < node.size() ; i++) 
      {
        parseLow(node[i],n);
      }
    }
    else if(node.Type() == YAML::NodeType::Map)
    {
      std::string s;
      for(YAML::Iterator it = node.begin() ; it != node.end() ; it++) 
      {
        Node* new_node = new Node();
        it.first() >> s;
        new_node->tag = s;
        new_node->file_origin = n->file_origin;
        new_node->file_row = node.GetMark().line;
        n->elements.push_back(new_node);
        if(s == "filename")
        {
          std::string file_name;
          it.second() >> file_name;
          std::string path = ros::package::getPath("stdr_resources") + 
              std::string("/resources/") + file_name;
          std::ifstream fin(path.c_str());
          #ifdef HAVE_NEW_YAMLCPP
            YAML::Node doc = YAML::Load(fin);
          #else
            YAML::Parser parser(fin);
            YAML::Node doc;
            parser.GetNextDocument(doc);
          #endif
          new_node->file_origin = file_name;
          new_node->file_row = doc.GetMark().line;
          
          parseLow(doc,new_node);
        }
        else
        {
          parseLow(it.second(),new_node);
        }
      }
    }
  }
}

