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

#include "stdr_parser/stdr_parser_xml_file_writer.h"

namespace stdr_parser
{
  
  /**
  @brief Default constructor
  @return void
  **/
  XmlFileWriter::XmlFileWriter(void)
  {

  }
  
  //!< Template declaration for stdr_msgs::FootprintMsg
  template void XmlFileWriter::messageToFile
    (stdr_msgs::FootprintMsg msg,std::string file_name);
  //!< Template declaration for stdr_msgs::Noise
  template void XmlFileWriter::messageToFile
    (stdr_msgs::Noise msg,std::string file_name);
  
  /**
  @brief Creates an xml file from a message - template member function
  @param msg [T] The message
  @param file_name [std::string] The xml file name to write the message
  @return void
  **/
  template <class T>
  void XmlFileWriter::messageToFile(T msg,std::string file_name)
  {
    TiXmlDocument doc;
    messageToXmlElement<T>(msg,&doc);
    doc.SaveFile((ros::package::getPath("stdr_resources") 
      + "/" + file_name).c_str()); 
  }
  
  //!< Template specialization for stdr_msgs::Noise
  template <>
  void XmlFileWriter::messageToXmlElement<stdr_msgs::Noise>
    (stdr_msgs::Noise msg,TiXmlNode* base){
    //!< Create noise
    TiXmlElement* noise;
    noise = new TiXmlElement("noise");
    base->LinkEndChild(noise);
        
    //!< Create noise specifications
    TiXmlElement* noise_specs;
    noise_specs = new TiXmlElement("noise_specifications");
    noise->LinkEndChild(noise_specs);
    
    //!< Create noise mean
    TiXmlElement* noise_mean;
    noise_mean = new TiXmlElement("noise_mean");
    noise_specs->LinkEndChild(noise_mean);
    
    TiXmlText * noise_mean_text = new TiXmlText(SSTR(msg.noiseMean));
    noise_mean->LinkEndChild(noise_mean_text);
    
    //!< Create noise std
    TiXmlElement* noise_std;
    noise_std = new TiXmlElement("noise_std");
    noise_specs->LinkEndChild(noise_std);
    
    TiXmlText * noise_std_text = new TiXmlText(SSTR(msg.noiseStd));
    noise_std->LinkEndChild(noise_std_text);
  }
  
  //!< Template specialization for stdr_msgs::Footprint
  template <>
  void XmlFileWriter::messageToXmlElement<stdr_msgs::FootprintMsg>
    (stdr_msgs::FootprintMsg msg,TiXmlNode* base){
    //!< Create noise
    TiXmlElement* footprint;
    footprint = new TiXmlElement("footprint");
    base->LinkEndChild(footprint);
        
    //!< Create footprint specifications
    TiXmlElement* footprint_specs;
    footprint_specs = new TiXmlElement("footprint_specifications");
    footprint->LinkEndChild(footprint_specs);
    
    //!< Create footprint radius
    TiXmlElement* radius;
    radius = new TiXmlElement("radius");
    footprint_specs->LinkEndChild(radius);
    
    TiXmlText * radius_text = new TiXmlText(SSTR(msg.radius));
    radius->LinkEndChild(radius_text);
  }
  
  
  /**
  @brief Creates an xml element from a msg - template member function
  @param msg [T] The message
  @param base [TiXmlNode*] The xml node to write the message
  @return void
  **/
  template <class T>
  void XmlFileWriter::messageToXmlElement(T msg,TiXmlNode* base)
  {
  }
}

