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

#ifndef STDR_XML_PARSER__NODE
#define STDR_XML_PARSER__NODE

#include "stdr_xml_parser/stdr_xml_parser_specs.h"

namespace stdr_xml_parser
{
  
  class Node
  {
    private:

    public:
      Node(void);
      bool check_for_filename(std::string base);
      std::vector<int> getTag(std::string tag);
      void increasePriority(void);
      
      int priority;
      std::string tag;
      std::string value;
      std::vector<Node*> elements;
  };
  
}
#endif
