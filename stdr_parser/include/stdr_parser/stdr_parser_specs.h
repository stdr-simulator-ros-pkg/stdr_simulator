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

#ifndef STDR_PARSER_SPECS
#define STDR_PARSER_SPECS

#include "stdr_parser/stdr_parser_tools.h"

/**
@namespace stdr_parser
@brief The main namespace for STDR parser
**/ 
namespace stdr_parser
{
  /**
  @struct ElSpecs
  @brief An element of Specs - represents a valid tag
  **/ 
  struct ElSpecs
  {
    /**
    @brief Default constructor
    @return void
    **/
    ElSpecs(void);
    
    //!< The required tags for the tag
    std::set<std::string> required;
    //!< The allowed tags for the tag
    std::set<std::string> allowed;
    //!< Default value for the node (if it is a value)
    std::string default_value;
  };
  
  /**
  @struct Specs
  @brief The STDR parser specifications
  **/ 
  struct Specs
  {
    /**
    @brief Default constructor
    @return void
    **/
    Specs(void);
    
    //!< std::map of valid STDR tags
    static std::map<std::string,ElSpecs> specs;
    
    //!< List of non-mergable tags. Read from stdr_multiple_allowed.xml
    static std::set<std::string> non_mergable_tags;
  };
}
#endif
