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
  
#include "stdr_gui/stdr_map_metainformation/stdr_gui_source.h"

namespace stdr_gui{

  /**
  @brief Default contructor
  @param p [QPoint] The pose of the rfid tag
  @param name [std::string] The "name" of the rfid tag
  @return void
  **/
  CGuiSource::CGuiSource(QPoint p,std::string name, float resolution):
    position_(p),
    name_(name),
    resolution_(resolution)
  {

  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiSource::~CGuiSource(void)
  {
    
  }

  /**
  @brief Returns the "name" of the source
  @return std::string 
  **/
  std::string CGuiSource::getName(void)
  {
    return name_;
  }
  
  /**
  @brief Checks proximity to a point
  @param p [QPoint] The proximity point to check
  @return bool : True if tag is close to p
  **/
  bool CGuiSource::checkProximity(QPoint p)
  {
    float dx = p.x() * resolution_ - position_.x() * resolution_;
    float dy = p.y() * resolution_ - position_.y() * resolution_;
    float dist = sqrt( pow(dx,2) + pow(dy,2) );
    return dist <= 0.3;
  }

}

