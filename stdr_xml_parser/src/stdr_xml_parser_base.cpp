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

#include "stdr_xml_parser/stdr_xml_parser_base.h"

namespace stdr_xml_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  Base::Base(void)
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
  void Base::printParsedXml(Node *n,std::string indent)
  {
    if(n->value != "")
    {  
      ROS_ERROR("%s- '%s' (%d)",indent.c_str(),n->value.c_str(),n->priority);
    }
    else
    {
      ROS_ERROR("%s[%s] (%d)",indent.c_str(),n->tag.c_str(),n->priority);
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
  void Base::printParsedXml(void)
  {
    ROS_ERROR("-----------------------------------------------------");
    printParsedXml(base_node_,"");
  }
  
  /**
  @brief High level function that eliminates the 'filename' nodes. Calls the eliminateFilenames(Node* n) function until it returns false
  @return void
  **/
  void Base::eliminateFilenames(void)
  {
    while(!eliminateFilenames(base_node_))
    {
    }
  }
  
  /**
  @brief Recursive function - Expands the 'filename' nodes and eliminates them
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is a 'filename' node was expanded
  **/
  bool Base::eliminateFilenames(Node* n)
  {
    if(n->value != "")
    {  
      return true;
    }
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      if(n->elements[i]->tag == "filename")
      { //!< Sanity check for filename. Base and file must be the same
        if(!n->elements[i]->check_for_filename(n->tag))
        {
          ROS_ERROR("STDR XML parser Error : [%s] had a filename of wrong \
type specified",n->tag.c_str());
          return true;
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
        if(!eliminateFilenames(n->elements[i]))
        {
          return false;
        }
      }
    }
    return true;
  }
  
  /**
  @brief Performs a allowed - validity check on the xml tree
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is the xml is allowed-valid
  **/
  bool Base::validityAllowedCheck(Node* n)
  {
    //!< Immediately return if we have a value node
    if(n->value != "")
    {
      return true;
    }
    std::string tag = n->tag;
    if(Specs::specs.find(tag) == Specs::specs.end() &&
      tag != "STDR_Parser_Root_Node")
    {
      ROS_ERROR("[%s] is not a valid tag",tag.c_str());
      return false;
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
          ROS_ERROR("[%s] is not allowed in a [%s] tag",
            child_tag.c_str(),tag.c_str());
          return false;
        }
      }
      if(!validityAllowedCheck(n->elements[i]))
      {
        return false;
      }
    }
    return true;
  }
  
  /**
  @brief Performs a required - validity check on the xml tree
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is the xml is required-valid
  **/
  bool Base::validityRequiredCheck(Node* n)
  {
    //!< Immediately return if we have a value node
    if(n->value != "")
    {
      return true;
    }
    std::string tag = n->tag;
    if(Specs::specs.find(tag) == Specs::specs.end() &&
      tag != "STDR_Parser_Root_Node")
    {
      ROS_ERROR("[%s] is not a valid tag",tag.c_str());
      return false;
    }
    for(std::set<std::string>::iterator it = Specs::specs[tag].required.begin() 
      ; it != Specs::specs[tag].required.end() ; it++)
    {
      std::vector<int> num = n->getTag(*it);
      if(num.size() == 0)
      {
        ROS_ERROR("[%s] requires [%s]",tag.c_str(),(*it).c_str());
        return false;
      }
    }
    for(unsigned int i = 0 ; i < n->elements.size() ; i++)
    {
      if(!validityRequiredCheck(n->elements[i]))
      {
        return false;
      }
    }
    return true;
  }
  /**
  @brief Merges the leaves of the xml tree, which are the value nodes
  @param n [Node*] The stdr xml tree node to begin
  @return void
  **/
  void Base::mergeNodesValues(Node* n)
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
  void Base::mergeNodes(void)
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
  bool Base::mergeNodes(Node* n)
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
  void Base::parse(std::string file_name)
  {
    // Must destroy prev tree
    parse(file_name,base_node_);
    
    parseMergableSpecifications();

    eliminateFilenames();
    
    mergeNodes();
    
    validityAllowedCheck(base_node_);
    
    validityRequiredCheck(base_node_);
  }
  
  /**
  @brief Private function that initiates the parsing of an xml file
  @param file_name [std::string] The xml file name
  @param n [Node*] The stdr xml tree node to update
  @return void
  **/
  void Base::parse(std::string file_name,Node* n)
  {
    std::string path=base_path_ + std::string("/xmls/") + file_name;
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      ROS_ERROR("Failed to load file \"%s\"\n,%s", 
        path.c_str(),doc.ErrorDesc());
    }
    parseLow(&doc,n);
  }
  
  /**
  @brief Parses the mergabl speciications file
  @return void
  **/
  void Base::parseMergableSpecifications(void)
  {
    std::string path=base_path_ + 
      std::string("/xmls/stdr_specific/stdr_multiple_allowed.xml");
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      ROS_ERROR("Failed to load file \"%s\"\n,%s", 
        path.c_str(),doc.ErrorDesc());
      return;
    }
    non_mergable_tags_ = explodeString(
      doc.FirstChild()->FirstChild()->Value(), ',');
  }
  
  /**
  @brief Low-level recursive function for parsing the xml robot file
  @param node [TiXmlNode*] The xml node to start from
  @param n [Node*] The stdr xml tree node to update
  @return void
  **/
  void Base::parseLow(TiXmlNode* node,Node* n)
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
        n->elements.push_back(new_node);
        break;
      }
      case 4 :    //!< Type = text
      {
        
        if(std::string(node->Parent()->Value()) == "filename")
        {
          parse(std::string(node->Value()) , n);
        }
        else
        {
          new_node->value = node_text;
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
  @brief Loads the specifications file and parses it
  @return void
  **/
  void Base::initialize(void)
  {
    std::string path=base_path_ + 
      std::string("/xmls/stdr_specific/stdr_specifications.xml");
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      ROS_ERROR("Failed to load specifications file.\nShould be at '%s'.\n\
Error was '%s'", path.c_str(),doc.ErrorDesc());
      ros::shutdown();
      exit(0);
    }
    
    parseSpecifications(&doc);
  }
  
  /**
  @brief Low-level recursive function for parsing the xml specifications file
  @param node [TiXmlNode*] The xml node to start from
  @return void
  **/
  void Base::parseSpecifications(TiXmlNode* node)
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
            ROS_ERROR("STDR XML parser Error : Multiple instance of %s in \
specifications.xml", node_text.c_str());
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
          ROS_ERROR("STDR XML parser Error : Specifications not in allowed \
or required : [%s] (%s) {%s}",base_tag.c_str(), base_type.c_str(), 
            node_text.c_str());
        }
        break;
      }
    }
    
    for ( 
      pChild = node->FirstChild(); 
      pChild != 0; 
      pChild = pChild->NextSibling()) 
    {
      parseSpecifications( pChild );
    }
  }
  
  /**
  @brief Prints the specifications
  @return void
  **/
  void Base::printSpecifications(void)
  {
    for(std::map<std::string,ElSpecs>::iterator it = Specs::specs.begin();
      it != Specs::specs.end() ; it ++)
    {
      ROS_ERROR("[%s]",it->first.c_str());
      ROS_ERROR("  [allowed : %s]",it->first.c_str());
      for(std::set<std::string>::iterator inner_it = 
        it->second.allowed.begin(); 
          inner_it != it->second.allowed.end() ; inner_it ++)
      {
        ROS_ERROR("    - %s",(*inner_it).c_str());
      }
      ROS_ERROR("  [required : %s]",it->first.c_str());
      for(std::set<std::string>::iterator inner_it = 
        it->second.required.begin(); 
          inner_it != it->second.required.end() ; inner_it ++)
      {
        ROS_ERROR("    - %s",(*inner_it).c_str());
      }
    }
  }
  
  /**
  @brief Parses an xml file and produces a stdr_msgs::RobotMsg message
  @param file_name [std::string] The xml filename
  @return stdr_msgs::RobotMsg : The robot message
  **/
  stdr_msgs::RobotMsg Base::createRobotMessage(std::string file_name)
  {
    initialize();
    parse(file_name);
    return creator_.createRobotMessage(base_node_);
  }
}

