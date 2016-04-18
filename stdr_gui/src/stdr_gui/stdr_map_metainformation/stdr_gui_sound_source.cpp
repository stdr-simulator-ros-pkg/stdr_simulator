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
  
#include "stdr_gui/stdr_map_metainformation/stdr_gui_sound_source.h"

namespace stdr_gui{
  
  /**
  @brief Default contructor
  @param p [QPoint] The pose of the rfid tag
  @param name [std::string] The "name" of the rfid tag
  @return void
  **/
  CGuiSoundSource::CGuiSoundSource(QPoint p,std::string name, float resolution):
    CGuiSource(p,name,resolution),
    db_(0.0)
  {
  
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiSoundSource::~CGuiSoundSource(void)
  {

  }

  /**
  @brief Draws the tag in the map
  @param img [QImage*] The image to draw to
  @return void
  **/
  void CGuiSoundSource::draw(QImage *img)
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

    //!< Draws the label
    
    int text_size = name_.size();
    
    //~ painter.setPen(QColor(0,0,0,100 * (2 - visualization_status_)));
    painter.setPen(QColor(0,0,0,100 * (2)));
    
    painter.drawRect(
      position_.x() + 10,
      img->height() - position_.y() - 30,
      3 + text_size * 9,
      20);
    
    //~ painter.setPen(QColor(255,255,255,100 * (2 - visualization_status_)));
    painter.setPen(QColor(255,255,255,100 * (2)));
    
    painter.fillRect(
      position_.x() + 10,
      img->height() - position_.y() - 30,
      3 + text_size * 9,
      20,
      QBrush(QColor(0,0,0,100 * (2))));
      //~ QBrush(QColor(0,0,0,100 * (2 - visualization_status_))));
    
    painter.setFont(QFont("Courier New"));
    painter.drawText(
      position_.x() + 12,
      img->height() - position_.y() - 15,
      QString(name_.c_str()));
  }
  
  /**
  @brief Sets the tag message
  @param msg [QString] The message to be set
  @return void
  **/
  void CGuiSoundSource::setDb(float db)
  {
    db_ = db;
  }
  
  /**
  @brief Returns the tag message
  @return QString
  **/
  float CGuiSoundSource::getDb(void)
  {
    return db_;
  }
}

