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

#include "stdr_parser/stdr_parser_node.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  Node::Node(void)
  {
    priority = 0;
  }
  
  Node::~Node(void)
  {
	this->unallocateChildren();
  }

  /**
  @brief Checks a node if a specific filename exists
  @return void
  **/
  bool Node::checkForFilename(std::string base)
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
  
  /**
  @brief Searches for a tag in the specific node
  @param tag [std::string] The tag to search for
  @return std::vector<int> : The indexes in elements where tag is found
  **/
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
  
  /**
  @brief Increases the priority of the node
  @return void
  **/
  void Node::increasePriority(void)
  {
    priority ++;
    for(unsigned int i = 0 ; i < elements.size() ; i++)
    {
      elements[i]->increasePriority();
    }
  }
  
  /**
  @brief Debug recursive function - Prints the xml tree
  @param n [Node*] The stdr xml tree node to begin
  @param indent [std::string] The indentation for the specific node
  @return void
  **/
  void Node::printParsedXml(Node *n,std::string indent)
  {
    if(n->value != "")
    {  
      ROS_ERROR("%s- '%s' (%d) - %d %s",indent.c_str(),n->value.c_str(),
        n->priority, n->file_row, extractFilename(n->file_origin).c_str());
    }
    else
    {
      ROS_ERROR("%s[%s] (%d) - %d %s",indent.c_str(),n->tag.c_str(),
        n->priority, n->file_row, extractFilename(n->file_origin).c_str());
    }  
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      printParsedXml(n->elements[i],indent+std::string("| "));
    }
  }
  
  /**
  * @brief Unallocates the memory of the node's children
  * @return void
  */
  void Node::unallocateChildren(void)
  {
    for(unsigned int i = 0 ; i < elements.size() ; i++)
    {

      delete elements[i];
    }
  }
}
