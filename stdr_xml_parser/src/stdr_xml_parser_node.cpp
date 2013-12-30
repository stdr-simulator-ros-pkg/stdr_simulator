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

#include "stdr_xml_parser/stdr_xml_parser_node.h"

namespace stdr_xml_parser
{
  
  Node::Node(void)
  {
    priority = 0;
  }
  
  bool Node::check_for_filename(std::string base)
  {
    if(elements.size() == 1)
    {
      if(elements[0]->tag == base)
      {
        return true;
      }
    }
    return false;
  }
  
  std::vector<int> Node::getTag(std::string tag)
  {
    std::vector<int> ret;
    for(unsigned int i = 0 ; i < elements.size() ; i++)
    {
      if(elements[i]->tag == tag)
      {
        ret.push_back(i);
      }
    }
    return ret;
  }
  
  void Node::increasePriority(void)
  {
    priority ++;
    for(unsigned int i = 0 ; i < elements.size() ; i++)
    {
      elements[i]->increasePriority();
    }
  }
}
