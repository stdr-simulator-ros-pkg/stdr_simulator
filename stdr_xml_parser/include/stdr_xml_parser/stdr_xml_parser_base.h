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

#ifndef STDR_XML_PARSER__BASE
#define STDR_XML_PARSER__BASE

#include "stdr_xml_parser/stdr_xml_parser_node.h"

namespace stdr_xml_parser
{
  
  class Base
  {
    private:
      Node* base_node_;
      Node* mergable_node_;
      std::string base_path_;
      std::set<std::string> mergable_tags_;
      
      void parseSpecifications(TiXmlNode* node);
      void loadSpecifications(void);
      std::set<std::string> explodeString(std::string s,char delimiter);
      void parseLow(TiXmlNode* node,Node* n);
      void parse(std::string file_name,Node* n);
      void printParsedXml(Node* n,std::string indent);
      bool eliminateFilenames(Node* n);
      void eliminateFilenames(void);
      void parseMergableSpecifications(void);
    public:
    
      Base(void);
      void initialize(void);
      void parse(std::string file_name);
      void printSpecifications(void);
      void printParsedXml(void);
  };
}
#endif
