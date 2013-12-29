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

#include "stdr_xml_parser/stdr_xml_parser.h"

namespace stdr_xml_parser
{
  //-----------------------------------------------------------------------//
  ElSpecs::ElSpecs(void)
  {
    required.clear();
    allowed.clear();
  }
  
  std::map<std::string,ElSpecs> Specs::specs = 
    std::map<std::string,ElSpecs>();
    
  Specs::Specs(void)
  {
    specs.clear();
  }
  //-----------------------------------------------------------------------//
  
  Base::Base(void)
  {
    base_path_=ros::package::getPath("stdr_resources");
  }
  
  void Base::initializeParsing(std::string file_name)
  {
    std::string path=base_path_ + std::string("/") + file_name;
    TiXmlDocument doc;
    //~ bool loadOkay = doc.LoadFile(path.c_str());
    //~ if (loadOkay)
    //~ {
      //~ printf("\n%s:\n", path.c_str());
    //~ }
    //~ else
    //~ {
      //~ printf("Failed to load file \"%s\"\n,%s", path.c_str(),doc.ErrorDesc());
    //~ }
    ros::shutdown();
    exit(0);
  }
  
  void Base::loadSpecifications(void)
  {
    //~ std::string path=base_path_ + 
      //~ std::string("/xmls/stdr_specifications.xml");
    //~ TiXmlDocument doc;
    //~ bool loadOkay = doc.LoadFile(path.c_str());
    //~ if (loadOkay)
    //~ {
      //~ ROS_ERROR("Specifications file found : %s", path.c_str());
    //~ }
    //~ else
    //~ {
      //~ ROS_ERROR("Failed to load specifications file.\nShould be at '%s'.\n\
//~ Error was '%s'", path.c_str(),doc.ErrorDesc());
      //~ ros::shutdown();
      //~ exit(0);
    //~ }
    //~ 
    //~ parseSpecifications(&doc);
  }
  
  void Base::parseSpecifications(TiXmlNode* pParent)
  {
    //~ TiXmlNode* pChild;
    //~ TiXmlText* pText;
    //~ int type = pParent->Type();
    //~ switch (type)
    //~ {
      //~ case 0 :    //!< Type = document
      //~ {
        //~ break;
      //~ }
      //~ case 1 :    //!< Type = element
      //~ {
        //~ 
        //~ break;
      //~ }
      //~ case 4 :    //!< Type = text
      //~ {
        //~ 
        //~ break;
      //~ }
    //~ }
    //~ ROS_ERROR("parsing [%d] %s",type,pParent->Value());
    //~ for ( pChild = pParent->FirstChild(); pChild != 0; 
      //~ pChild = pChild->NextSibling()) 
    //~ {
      //~ parseSpecifications( pChild );
    //~ }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"stdr_xml_parser");
  
  stdr_xml_parser::Base b;
  b.loadSpecifications();
  ROS_ERROR("Parsing done");
  ros::spin();
  return 0;
}
