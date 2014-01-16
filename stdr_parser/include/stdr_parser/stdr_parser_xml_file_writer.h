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

#ifndef STDR_PARSER_XML_FILE_WRITER
#define STDR_PARSER_XML_FILE_WRITER

#include "stdr_parser/stdr_parser_node.h"

/**
@namespace stdr_parser
@brief The main namespace for STDR parser
**/ 
namespace stdr_parser
{
  /**
  @class FileWriter
  @brief Writes a node tree to an xml file
  **/ 
  class XmlFileWriter
  {
    private:
     
    public:
    
      /**
      @brief Default constructor
      @return void
      **/
      XmlFileWriter(void);
      
      /**
      @brief Creates an xml file from a noise msg
      @param msg [stdr_msgs::Noise] The noise message
      @param file_name [std::string] The xml file name to write the message
      @return void
      **/
      void noiseToFile(stdr_msgs::Noise msg,std::string file_name);
      
      /**
      @brief Creates an xml element from a noise msg
      @param msg [stdr_msgs::Noise] The noise message
      @param base [TiXmlNode*] The xml node to write the message
      @return void
      **/
      void noiseToXmlElement(stdr_msgs::Noise msg,TiXmlNode* base);
      
      //!<-------------------------------------------------------------
      
      /**
      @brief Creates an xml file from a footprint msg
      @param msg [stdr_msgs::FootprintMsg] The footprint message
      @param file_name [std::string] The xml file name to write the message
      @return void
      **/
      void footprintToFile(stdr_msgs::FootprintMsg msg,std::string file_name);
      
      /**
      @brief Creates an xml element from a footprint msg
      @param msg [stdr_msgs::FootprintMsg] The footprint message
      @param base [TiXmlNode*] The xml node to write the message
      @return void
      **/
      void footprintToXmlElement(stdr_msgs::FootprintMsg msg,TiXmlNode* base);
  };
}
#endif
