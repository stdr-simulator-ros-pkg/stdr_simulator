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

#ifndef HELPER_H
#define HELPER_H


#define PI 3.141592653589793

namespace stdr_robot {

/**
  @brief Checks if an angle is between two others. Supposes that min < max
  @param target_ [float] The target angle
  @param min_ [float] min angle
  @param max_ [float] max angle
  @return true on success
  **/ 
  static bool angCheck(float target_, float min_, float max_) 
  {
    int c = 0;
    c = (target_ + 2 * PI) / (2 * PI);
    float target = target_ + (1 - c) * PI * 2;
    c = (min_ + 2 * PI) / (2 * PI);
    float min = min_ + (1 - c) * PI * 2;
    c = (max_ + 2 * PI) / (2 * PI);
    float max = max_ + (1 - c) * PI * 2;
    
    if(min_ * max_ > 0) //!< Same sign
    {
      if(target > min && target < max)
      {
        return true;
      }
    }
    else
    {
      max += 2 * PI;
      if(target > min && target < max)
      {
        return true;
      }
      target += 2 * PI;
      if(target > min && target < max)
      {
        return true;
      }
    }
    return false;
  }

}  // namespace stdr_robot

#endif
