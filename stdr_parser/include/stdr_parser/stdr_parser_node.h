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

#ifndef STDR_PARSER_NODE
#define STDR_PARSER_NODE

#include "stdr_parser/stdr_parser_specs.h"

/**
@namespace stdr_parser
@brief The main namespace for STDR parser
**/ 
namespace stdr_parser
{
  /**
  @class Node
  @brief Implements the main functionalities of the stdr parser tree
  **/ 
  class Node
  {
    private:
      
      /**
       * @brief Unalloates the memory of the node's children
       * @return void
       */
      void unallocateChildren(void);

    public:
    
      /**
      @brief Default constructor
      @return void
      **/
      Node(void);

      /**
      @brief Destructor who also destroys all its children.
      @return void
      **/ 
      ~Node(void);

      /**
      @brief Checks a node if a specific filename exists
      @return void
      **/
      bool checkForFilename(std::string base);
      
      /**
      @brief Searches for a tag in the specific node
      @param tag [std::string] The tag to search for
      @return std::vector<int> : The indexes in elements where tag is found
      **/
      std::vector<int> getTag(std::string tag);
      
      /**
      @brief Increases the priority of the node
      @return void
      **/
      void increasePriority(void);
      
      //!< The node priority. Used in merging. Basically works inversely. Nodes in lower priority overwrite the ones in higher priority
      int priority;
      
      //!< The node tag (if it not a value node)
      std::string tag;
      //!< The node value (if it not a tag node)
      std::string value;
      
      //!< The node children
      std::vector<Node*> elements;
      
      //!< File it was into
      std::string file_origin;
      
      //!< Row in the original file
      int file_row;
      
      /**
      @brief Debug recursive function - Prints the xml tree
      @param n [Node*] The stdr xml tree node to begin
      @param indent [std::string] The indentation for the specific node
      @return void
      **/
      void printParsedXml(Node *n,std::string indent);

  };
  
}
#endif
