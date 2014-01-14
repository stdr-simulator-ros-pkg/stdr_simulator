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

#include "stdr_parser/stdr_parser.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  Parser::Parser(void)
  {
    base_path_=ros::package::getPath("stdr_resources");
    base_node_ = new Node();
    base_node_->tag = "STDR_Parser_Root_Node";
  }
  
  /**
  @brief Debug recursive function - Prints the xml tree
  @param n [Node*] The stdr xml tree node to begin
  @param indent [std::string] The indentation for the specific node
  @return void
  **/
  void Parser::printParsedXml(Node *n,std::string indent)
  {
    if(n->value != "")
    {  
      ROS_ERROR("%s- '%s' (%d)",indent.c_str(),n->value.c_str(),
        n->priority);
    }
    else
    {
      ROS_ERROR("%s[%s] (%d)",indent.c_str(),n->tag.c_str(),
        n->priority);
    }  
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      printParsedXml(n->elements[i],indent+std::string("| "));
    }
  }
  
  /**
  @brief Prints the parsed xml
  @return void
  **/
  void Parser::printParsedXml(void)
  {
    ROS_ERROR("-----------------------------------------------------");
    printParsedXml(base_node_,"");
  }
  
  /**
  @brief High level function that eliminates the 'filename' nodes. Calls the eliminateFilenames(Node* n) function until it returns false
  @return void
  **/
  void Parser::eliminateFilenames(void)
  {
    try
    {
      while(!eliminateFilenames(base_node_))
      {
      }
    }
    catch(ParserException ex)
    {
      throw ex;
    }
  }
  
  /**
  @brief Recursive function - Expands the 'filename' nodes and eliminates them
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is a 'filename' node was expanded
  **/
  bool Parser::eliminateFilenames(Node* n)
  {
    if(n->value != "")
    {  
      return true;
    }
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      if(n->elements[i]->tag == "filename")
      { //!< Sanity check for filename. Base and file must be the same
        if(!n->elements[i]->checkForFilename(n->tag))
        {
          std::string error = 
            std::string("STDR parser : ") + n->tag + std::string(" has a \
filename of wrong type specified\n") + 
            std::string("\nError was in line ") + SSTR( n->file_row ) + 
            std::string(" of file '") + n->file_origin + std::string("'");
          throw ParserException(error);
        }
        Node* child = n->elements[i]->elements[0];
        n->elements.erase(n->elements.begin() + i);
        for(unsigned int j = 0 ; j < child->elements.size() ; j++)
        {
          child->elements[j]->increasePriority() ;
          n->elements.push_back(child->elements[j]);
        }
        return false;
      }
      else
      {
        try
        {
          if(!eliminateFilenames(n->elements[i]))
          {
            return false;
          }
        }
        catch(ParserException ex)
        {
          throw ex;
        }
      }
    }
    return true;
  }
  
  /**
  @brief Merges the leaves of the xml tree, which are the value nodes
  @param n [Node*] The stdr xml tree node to begin
  @return void
  **/
  void Parser::mergeNodesValues(Node* n)
  {
    //!< Check if the node does not contain pure values
    bool pure_values = true;
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      if(n->elements[i]->value == "")
      {
        pure_values = false;
        break;
      }
    }
    
    if(pure_values)
    {
      //!< Unique value child
      if(n->elements.size() <= 1)
      {
        return;
      }
      
      //!< Multiple value childer. Find min priority
      int min_priority = n->elements[0]->priority;
      unsigned int index = 0;
      for(unsigned int i = 1 ; i < n->elements.size() ; i++)
      {
        if(n->elements[i]->priority < min_priority)
        {
          min_priority = n->elements[i]->priority;
          index = i;
        }
      }
      Node* proper_child = n->elements[index];
      n->elements.clear();
      n->elements.push_back(proper_child);
    }
    else
    {
      for(unsigned int i = 0 ; i < n->elements.size() ; i++)
      {
        mergeNodesValues(n->elements[i]);
      }
    }
  }
  
  /**
  @brief High level function that merges similar nodes. Calls the mergeNodes(Node* n) function until it returns false
  @return void
  **/
  void Parser::mergeNodes(void)
  {
    while(!mergeNodes(base_node_))
    {
    }
    mergeNodesValues(base_node_);
  }

  /**
  @brief Recursive function - Merges the nodes that do not exist in non_mergable_tags_
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is a ndoe was merged
  **/
  bool Parser::mergeNodes(Node* n)
  {
    if(n->value != "")  //!< Node is value
    {
      return true;
    }
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      //!< Node's child is value
      if(n->elements[i]->value != "") 
      {
        continue;
      }
      std::string tag = n->elements[i]->tag;
      
      //!< Child is a mergable tag
      if(non_mergable_tags_.find(tag) == non_mergable_tags_.end())
      {
        std::vector<int> num = n->getTag(tag);
        
        //!< Multiple mergable tags
        if(num.size() != 1) 
        { 
          for(int i = num.size()-1 ; i > 0 ; i --)
          {
            //!< Merging multiple tag's children into the first occurence
            for (unsigned int j = 0 ; 
              j < n->elements[num[i]]->elements.size() ; j++)
            {
              n->elements[num[0]]->elements.push_back(
                n->elements[num[i]]->elements[j]);
            }
            n->elements.erase(n->elements.begin() + num[i]);
          }
          return false;
        }
      }
      
      if(!mergeNodes(n->elements[i]))
      {
        return false;
      }
    }
    return true;
  }
  
  /**
  @brief Parses an xml file
  @param file_name [std::string] The xml filename
  @return void
  **/
  void Parser::parse(std::string file_name)
  {
    // Must destroy prev tree
    try
    {
      parse(file_name,base_node_);  
      eliminateFilenames(); 
      mergeNodes();
      validator_.validate(base_node_);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
  }
  
  /**
  @brief Private function that initiates the parsing of an xml file
  @param file_name [std::string] The xml file name
  @param n [Node*] The stdr xml tree node to update
  @return void
  **/
  void Parser::parse(std::string file_name,Node* n)
  {
    std::string path=base_path_ + std::string("/xmls/") + file_name;
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      std::string error = 
        std::string("STDR parser : Failed to load file '") + 
        path + std::string("'") +
        std::string("\nError was '") + std::string(doc.ErrorDesc()) + 
        std::string("'\nIf error was 'Error reading end tag' you have a \
malformed xml file");
      throw ParserException(error);
    }
    n->file_origin = path;
    n->file_row = doc.Row();
    parseLow(&doc,n);
  }
  
  /**
  @brief Low-level recursive function for parsing the xml robot file
  @param node [TiXmlNode*] The xml node to start from
  @param n [Node*] The stdr xml tree node to update
  @return void
  **/
  void Parser::parseLow(TiXmlNode* node,Node* n)
  {
    Node* new_node = new Node();
    TiXmlNode* pChild;
    int type = node->Type();
    std::string node_text(node->Value());
    switch (type)
    {
      case 0 :    //!< Type = document
      {
        new_node = n;
        break;
      }
      case 1 :    //!< Type = element
      {
        new_node->tag = node_text;
        new_node->file_origin = n->file_origin;
        n->file_row = node->Row();
        n->elements.push_back(new_node);
        break;
      }
      case 4 :    //!< Type = text
      {
        
        if(std::string(node->Parent()->Value()) == "filename")
        {
          try
          {
            parse(std::string(node->Value()) , n);
          }
          catch(ParserException ex)
          {
            throw ex;
          }
          
        }
        else
        {
          new_node->value = node_text;
          new_node->file_origin = n->file_origin;
          n->file_row = node->Row();
          n->elements.push_back(new_node);
        }
        break;
      }
    }
    
    for ( 
      pChild = node->FirstChild(); 
      pChild != 0; 
      pChild = pChild->NextSibling()) 
    {
      parseLow( pChild , new_node );
    }
  }
  
  /**
  @brief Parses an xml file and produces a stdr_msgs::RobotMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::RobotMsg : The robot message
  **/
  stdr_msgs::RobotMsg Parser::createRobotMessage(std::string file_name)
  {
    
    try
    {
      parse(file_name);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    return creator_.createRobotMessage(base_node_);
  }
  
  /**
  @brief Parses an xml file and produces a stdr_msgs::LaserSensorMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::LaserSensorMsg : The laser message
  **/
  stdr_msgs::LaserSensorMsg Parser::createLaserMessage(std::string file_name)
  {
    
    try
    {
      parse(file_name);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    
    return creator_.createLaserMessage(base_node_,0);
  }
  
  /**
  @brief Parses an xml file and produces a stdr_msgs::SonarSensorMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::SonarSensorMsg : The sonar message
  **/
  stdr_msgs::SonarSensorMsg Parser::createSonarMessage(std::string file_name)
  {
    
    try
    {
      parse(file_name);
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    return creator_.createSonarMessage(base_node_,0);
  }
}

