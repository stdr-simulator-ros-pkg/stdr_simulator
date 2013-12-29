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
  
#include "stdr_gui/stdr_map_metainformation/stdr_gui_rfid_tag.h"

namespace stdr_gui{
  
  /**
  @brief Default contructor
  @param p [QPoint] The pose of the rfid tag
  @param name [std::string] The "name" of the rfid tag
  @return void
  **/
  CGuiRfidTag::CGuiRfidTag(QPoint p,std::string name):
    position_(p),
    name_(name),
    message_("")
  {
  
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiRfidTag::~CGuiRfidTag(void)
  {

  }
  
  /**
  @brief Returns the "name" of the rfid tag
  @return std::string 
  **/
  std::string CGuiRfidTag::getName(void)
  {
    return name_;
  }
  
  /**
  @brief Checks proximity to a point
  @param p [QPoint] The proximity point to check
  @return bool : True if tag is close to p
  **/
  bool CGuiRfidTag::checkProximity(QPoint p)
  {
    return false;  // 2b changed
  }
  
  /**
  @brief Draws the tag in the map
  @param img [QImage*] The image to draw to
  @return void
  **/
  void CGuiRfidTag::draw(QImage *img)
  {
    QPainter painter(img);
    int step = 3;
    painter.setPen(QColor(0,200,0,200));
    for(unsigned int i = 0 ; i < 4 ; i++)
    {
      painter.drawEllipse(
        position_.x() - i * step, 
        img->height() - position_.y() - i * step, 
        2 * i * step, 
        2 * i * step);
    }
  }
  
  /**
  @brief Sets the tag message
  @param msg [QString] The message to be set
  @return void
  **/
  void CGuiRfidTag::setMessage(QString msg)
  {
    message_ = msg;
  }
  
  /**
  @brief Returns the tag message
  @return QString
  **/
  QString CGuiRfidTag::getMessage(void)
  {
    return message_;
  }
}
