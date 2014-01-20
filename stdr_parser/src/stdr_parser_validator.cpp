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

#include "stdr_parser/stdr_parser_validator.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  Validator::Validator(void)
  {
    
  }
  
  /**
  @brief Performs a allowed - validity check on the xml tree
  @param file_name [std::string] The filename from which the node came from
  @param n [Node*] The stdr xml tree node to begin
  @return void
  **/
  void Validator::validityAllowedCheck(std::string file_name, Node* n)
  {
    //!< Immediately return if we have a value node
    if(n->value != "")
    {
      return;
    }
    std::string tag = n->tag;
    if(Specs::specs.find(tag) == Specs::specs.end() &&
      tag != "STDR_Parser_Root_Node")
    {
      std::string error = 
        std::string("STDR parser : ") + n->tag + 
        std::string(" is not a valid tag") + 
        std::string("\nTrail: ");
      throw ParserException(error);
    }
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      std::string child_tag = n->elements[i]->tag;
      std::string child_value = n->elements[i]->value;
      if(tag != "STDR_Parser_Root_Node" && child_value == "")
      {
        if(Specs::specs[tag].allowed.find(child_tag) == 
          Specs::specs[tag].allowed.end())
        {
          int decreaser = (extractFilename(n->elements[i]->file_origin) == 
          extractFilename(file_name) ? 1 : 0);
          std::string error = 
            std::string("STDR parser : ") + child_tag + 
            std::string(" is not allowed in a ") + tag + std::string(" tag") +
            std::string("\nTrail: ");
          error += std::string("\n  [") + child_tag + std::string("] Line ") + 
            SSTR( n->elements[i]->file_row - decreaser ) + 
            std::string(" of file '") + 
            extractFilename(n->elements[i]->file_origin) + 
            std::string("'");
          throw ParserException(error);
        }
      }
      try
      {
        validityAllowedCheck(file_name, n->elements[i]);
      }
      catch(ParserException ex)
      {
        if(n->tag == "STDR_Parser_Root_Node")
        {
          throw ex;
        }
        int decreaser = (extractFilename(n->file_origin) == 
          extractFilename(file_name) ? 1 : 0);
        std::string trail(ex.what());
        trail += std::string("\n  [") + n->tag + std::string("] Line ") + 
          SSTR( n->file_row - decreaser) + 
          std::string(" of file '") + 
          extractFilename(n->file_origin) + std::string("'");
        ParserException ex_new(trail);
        throw ex_new;
      }
    }
  }
  
  /**
  @brief Performs a required - validity check on the xml tree
  @param file_name [std::string] The filename from which the node came from
  @param n [Node*] The stdr xml tree node to begin
  @return void
  **/
  void Validator::validityRequiredCheck(std::string file_name, Node* n)
  {
    //!< Immediately return if we have a value node
    if(n->value != "")
    {
      return;
    }
    std::string tag = n->tag;
    if(Specs::specs.find(tag) == Specs::specs.end() &&
      tag != "STDR_Parser_Root_Node")
    {
      std::string error = 
        std::string("STDR parser : ") + n->tag + 
        std::string(" is not a valid tag") + 
        std::string("\nTrail: ");
      throw ParserException(error);
    }
    for(std::set<std::string>::iterator it = Specs::specs[tag].required.begin() 
      ; it != Specs::specs[tag].required.end() ; it++)
    {
      std::vector<int> num = n->getTag(*it);
      if(num.size() == 0)
      {
        std::string error = 
          std::string("STDR parser : ") + tag + 
          std::string(" requires ") + (*it) +
          std::string("\nTrail: ");
        throw ParserException(error);
      }
    }
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      try
      {
        validityRequiredCheck(file_name, n->elements[i]);  
      }
      catch(ParserException ex)
      {
        if(n->tag == "STDR_Parser_Root_Node")
        {
          throw ex;
        }
        
        int decreaser = (extractFilename(n->elements[i]->file_origin) == 
          extractFilename(file_name) ? 1 : 0);
        
        std::string trail(ex.what());
        trail += std::string("\n  [") + n->tag + std::string("] Line ") + 
          SSTR( n->file_row - decreaser ) + 
          std::string(" of file '") + extractFilename(n->file_origin) 
          + std::string("'");
        ParserException ex_new(trail);
        throw ex_new;
      }
    }
  }
  
  /**
  @brief Parses the mergable speciications file
  @return void
  **/
  void Validator::parseMergableSpecifications(void)
  {
    std::string base_path_ = ros::package::getPath("stdr_resources");
    std::string path=base_path_ + 
      std::string("/resources/specifications/stdr_multiple_allowed.xml");
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      std::string error = 
        std::string("STDR parser : Failed to load file ") + 
        path + std::string("'") +
        std::string("\nError was \n\t") + std::string(doc.ErrorDesc());
      throw ParserException(error);
    }
    Specs::non_mergable_tags = explodeString(
      doc.FirstChild()->FirstChild()->Value(), ',');
  }

  /**
  @brief Low-level recursive function for parsing the xml specifications file
  @param node [TiXmlNode*] The xml node to start from
  @return void
  **/
  void Validator::parseSpecifications(TiXmlNode* node)
  {
    TiXmlNode* pChild;
    int type = node->Type();
    std::string node_text(node->Value());
    switch (type)
    {
      case 0 :    //!< Type = document
      {
        break;
      }
      case 1 :    //!< Type = element
      {
        if(node_text == "specifications")
        {
          break;
        }
        else if(node_text!="allowed" && node_text!="required" 
          && node_text!="default")
        { //!< Base specifications tag
          if(Specs::specs.find(node_text) == Specs::specs.end())
          { //!< New base specifications tag
            Specs::specs.insert(std::pair<std::string,ElSpecs>(
              node_text,ElSpecs()));
          }
          else
          { //!< Multiple base tags - error
            std::string error = 
              std::string("STDR parser : Multiple instance of '") + node_text +
              std::string("' in specifications.");
            throw ParserException(error);
          }
        }
        else
        { //!< required or allowed tags
          break;
        }
        break;
      }
      case 4 :    //!< Type = text
      {
        std::string base_tag (node->Parent()->Parent()->Value());
        std::string base_type(node->Parent()->Value());
        if(base_type == "allowed")
        {
          Specs::specs[base_tag].allowed = explodeString(node_text,',');
        }
        else if(base_type == "required")
        {
          Specs::specs[base_tag].required = explodeString(node_text,',');
        }
        else if(base_type == "default")
        {
          Specs::specs[base_tag].default_value = node_text;
        }
        else
        {
          std::string error = 
            std::string("STDR parser : Specification '") + base_tag +
            std::string("' not in 'allowed', 'required' or 'default' : '") +
            base_type + std::string("' / '") + node_text  + std::string("'");
          throw ParserException(error);
        }
        break;
      }
    }
    
    for ( 
      pChild = node->FirstChild(); 
      pChild != 0; 
      pChild = pChild->NextSibling()) 
    {
      try
      {
        parseSpecifications( pChild );
      }
      catch(ParserException ex)
      {
        throw ex;
      }
    }
  }
  
  /**
  @brief Performs a required / allowed - validity check on the xml tree
  @param file_name [std::string] The filename from which the tree was created
  @param n [Node*] The stdr xml tree node to begin
  @return void
  **/
  void Validator::validate(std::string file_name, Node* n)
  {
    
    Specs::specs.clear();
    Specs::non_mergable_tags.clear();
    
    std::string base_path_ = ros::package::getPath("stdr_resources");
    
    std::string path = base_path_ + 
      std::string("/resources/specifications/stdr_specifications.xml");
    
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      std::string error = 
        std::string("Failed to load specifications file.\nShould be at '") + 
        path + std::string("'\nError was") + std::string(doc.ErrorDesc());
      throw ParserException(error);
    }
    
    try
    {
      parseSpecifications(&doc);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    
    try
    {
      validityAllowedCheck(file_name, n);
      validityRequiredCheck(file_name, n);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
  }
}

