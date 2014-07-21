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

#include <stdexcept>

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

namespace stdr_robot {

/**
@class ConnectionException
@brief Provides a connection exception. Publicly inherits from std::runtime_error. Used in robot handler.
**/ 
class ConnectionException : public std::runtime_error
{
  public:
    /**
    @brief Throws an std::runtime_error with a messsage
    @param errorDescription [const std::string] The error message
    **/ 
    ConnectionException(const std::string errorDescription) : 
      std::runtime_error(errorDescription) 
    {
    }

};
/**
@class DoubleFrameIdException
@brief Provides a double frame id exception. Publicly inherits from std::runtime_error. Used in robot handler.
**/ 
class DoubleFrameIdException : public std::runtime_error
{
  public:
    /**
    @brief Throws an std::runtime_error with a messsage
    @param errorDescription [const std::string] The error message
    **/ 
    DoubleFrameIdException(const std::string errorDescription) : 
      std::runtime_error(errorDescription) 
    {
    }

};
} // end of namespace stdr_robot

#endif
