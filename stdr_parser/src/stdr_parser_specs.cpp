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

#include "stdr_parser/stdr_parser_specs.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  ElSpecs::ElSpecs(void)
  {
    required.clear();
    allowed.clear();
  }
    
  //!< Static member variable initialization
  std::map<std::string,ElSpecs> Specs::specs = std::map<std::string,ElSpecs>();
  //!< List of non-mergable tags. Read from stdr_multiple_allowed.xml
  std::set<std::string> Specs::non_mergable_tags = std::set<std::string>();
  
  /**
  @brief Default constructor
  @return void
  **/
  Specs::Specs(void)
  {
    specs.clear();
  }
}
