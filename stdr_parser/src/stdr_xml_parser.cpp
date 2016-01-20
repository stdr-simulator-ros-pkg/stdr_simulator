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

#include "stdr_parser/stdr_xml_parser.h"

namespace stdr_parser
{
  
  /**
  @brief Default constructor
  @return void
  **/
  XmlParser::XmlParser(void)
  {

  }
  
  /**
  @brief Parses an xml file
  @param file_name [std::string] The xml filename
  @return void
  **/
  void XmlParser::parse(std::string file_name, Node* base_node)
  {
    // Must destroy prev tree
    std::string path = file_name;
    TiXmlDocument doc;
    doc.SetTabSize(2);
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      std::string error = 
        std::string("Failed to load file '") + 
        path + std::string("'") +
        std::string("\nError was '") + std::string(doc.ErrorDesc()) + 
        std::string("'\nIf error was 'Error reading end tag' you have a \
malformed xml file");
      throw ParserException(error);
    }
    base_node->file_origin = path;
    base_node->file_row = doc.Row();
    parseLow(&doc,base_node); 
  }
 
  /**
  @brief Low-level recursive function for parsing the xml robot file
  @param node [TiXmlNode*] The xml node to start from
  @param n [Node*] The stdr xml tree node to update
  @return void
  **/
  void XmlParser::parseLow(TiXmlNode* node, Node* n)
  {
    Node* new_node = new Node();
    TiXmlNode* pChild;
    int type = node->Type();
    std::string node_text(node->Value());
    switch (type)
    {
      case 0 :    //!< Type = document
      {
        new_node = n;
        break;
      }
      case 1 :    //!< Type = element
      {
        new_node->tag = node_text;
        new_node->file_origin = n->file_origin;
        n->file_row = node->Row();
        n->elements.push_back(new_node);
        break;
      }
      case 4 :    //!< Type = text
      {
        
        if(std::string(node->Parent()->Value()) == "filename")
        {
          try
          {
            parse(ros::package::getPath("stdr_resources") + 
              std::string("/resources/") + std::string(node->Value()), n);
          }
          catch(ParserException ex)
          {
            try
            {
              // If not found on stdr_resources/resources,
              // search on the directory containing parent file
              parse(extractDirname(new_node->file_origin) +
                std::string("/") + std::string(node->Value()), n);
            }
            catch(ParserException ex)
            {
              throw ex;
            }
          }
        }
        else
        {
          new_node->value = node_text;
          new_node->file_origin = n->file_origin;
          n->file_row = node->Row();
          n->elements.push_back(new_node);
        }
        break;
      }
    }
    
    for ( 
      pChild = node->FirstChild(); 
      pChild != 0; 
      pChild = pChild->NextSibling()) 
    {
      parseLow( pChild , new_node );
    }
  }
  
}

