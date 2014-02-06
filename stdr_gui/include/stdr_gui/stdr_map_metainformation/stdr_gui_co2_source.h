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
#ifndef STDR_GUI_CO2_SOURCE_CONTAINER
#define STDR_GUI_CO2_SOURCE_CONTAINER

#include "stdr_gui/stdr_tools.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CGuiCo2Source
  @brief Implements the functionalities of a CO2 source
  **/ 
  class CGuiCo2Source
  {
    //------------------------------------------------------------------------//
    private:
      //!< The position of the CO2 source in the map
      QPoint position_;  
      //!< The "name" of the CO2 source
      std::string name_;    
    //------------------------------------------------------------------------//
    public:
    
      /**
      @brief Default contructor
      @param p [QPoint] The pose of the CO2 source
      @param name [std::string] The "name" of the source
      @return void
      **/
      CGuiCo2Source(QPoint p,std::string name);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiCo2Source(void);
      
      /**
      @brief Returns the "name" of the CO2 source
      @return std::string 
      **/
      std::string getName(void);
      
      /**
      @brief Checks proximity to a point
      @param p [QPoint] The proximity point to check
      @return bool : True if CO2 source is close to p
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

