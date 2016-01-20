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

#include "stdr_parser/stdr_parser_tools.h"

namespace stdr_parser
{
  /**
  @brief Explodes a string based on a delimiter
  @param s [std::string] The input string
  @param delimiter [char] The delimiter
  @return std::set<std::string> : An ensemble of strings
  **/
  std::set<std::string> explodeString(std::string s,char delimiter)
  {
    std::set<std::string> ret;
    int prev = 0, next = 0;
    next = s.find(delimiter,prev);
    while(next != std::string::npos)
    {
      ret.insert(s.substr(prev , next - prev));
      prev = next + 1;
      next = s.find(delimiter,prev);
    }
    ret.insert(s.substr(prev , s.size() - prev));
    return ret;
  }
  
  /**
  @brief Extracts the filename from an absolute path
  @param s [std::string] The input string
  @return std::string
  **/
  std::string extractFilename(std::string s)
  {
    int n = s.find_last_of('/');
    return s.substr(n + 1, s.size() - n - 1);
  }

  /**
  @brief Extracts the directory from an absolute path. It does the
  same functionality as libgen's dirname but for std::string objects
  @param s [std::string] The input string
  @return std::string
  **/
  std::string extractDirname(std::string s)
  {
    int n = s.find_last_of('/');
    return s.substr(0, n); // exclude trailing '/'
  }
}
