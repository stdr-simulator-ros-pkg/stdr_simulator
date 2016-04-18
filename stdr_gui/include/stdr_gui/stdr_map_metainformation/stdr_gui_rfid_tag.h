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


#ifndef STDR_GUI_RFID_TAG_CONTAINER
#define STDR_GUI_RFID_TAG_CONTAINER

#include "stdr_gui/stdr_map_metainformation/stdr_gui_source.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CGuiRfidTag
  @brief Implements the functionalities of an RFID tag
  **/ 
  class CGuiRfidTag : public CGuiSource
  {
    //------------------------------------------------------------------------//
    private:
      //!< The message of the rfid tag
      QString message_;

    //------------------------------------------------------------------------//
    public:
      /**
      @brief Default contructor
      @param p [QPoint] The pose of the rfid tag
      @param name [std::string] The "name" of the rfid tag
      @param resolution [float] The map's resolution
      @return void
      **/
      CGuiRfidTag(QPoint p,std::string name, float resolution);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiRfidTag(void);
      
      /**
      @brief Draws the tag in the map
      @param img [QImage*] The image to draw to
      @return void
      **/
      virtual void draw(QImage *img);
      
      /**
      @brief Sets the tag message
      @param msg [QString] The message to be set
      @return void
      **/
      void setMessage(QString msg);
      
      /**
      @brief Returns the tag message
      @return QString
      **/
      QString getMessage(void);
  };  
}

#endif
