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


#ifndef STDR_GUI_THERMAL_SOURCE_CONTAINER
#define STDR_GUI_THERMAL_SOURCE_CONTAINER

#include "stdr_gui/stdr_tools.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CGuiThermalSource
  @brief Implements the functionalities of a thermal source
  **/ 
  class CGuiThermalSource
  {
    //------------------------------------------------------------------------//
    private:
      //!< The position of the thermal source in the map
      QPoint position_; 
      //!< The "name" of the thermal source   
      std::string name_;  
    //------------------------------------------------------------------------//
    public:
      /**
      @brief Default contructor
      @param p [QPoint] The pose of the thermal source
      @param name [std::string] The "name" of the source
      @return void
      **/
      CGuiThermalSource(QPoint p,std::string name);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiThermalSource(void);
      
      /**
      @brief Returns the "name" of the thermal source
      @return std::string 
      **/
      std::string getName(void);
      
      /**
      @brief Checks proximity to a point
      @param p [QPoint] The proximity point to check
      @return bool : True if thermal source is close to p
      **/
      bool checkProximity(QPoint p);
      
      /**
      @brief Draws the source in the map
      @param img [QImage*] The image to draw to
      @return void
      **/
      void draw(QImage *img);
  };  
}

#endif
