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

#include "stdr_gui/stdr_robot_creator/stdr_kinematic_properties_loader.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char**] Input arguments
  @return void
  **/
  CKinematicPropertiesLoader::CKinematicPropertiesLoader(
    int argc, char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CKinematicPropertiesLoader::~CKinematicPropertiesLoader(void)
  {
    
  }
}
