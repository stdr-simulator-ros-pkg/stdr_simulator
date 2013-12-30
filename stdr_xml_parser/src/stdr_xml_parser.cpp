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
  Node::Node(void)
  {
    priority = 0;
  }
  
  bool Node::check_for_filename(std::string base)
  {
    if(elements.size() == 1)
    {
      if(elements[0]->tag == base)
      {
        return true;
      }
    }
    return false;
  }
  
  std::vector<int> Node::get_tag(std::string tag)
  {
    std::vector<int> ret;
    for(unsigned int i = 0 ; i < elements.size() ; i++)
    {
      if(elements[i]->tag == tag)
      {
        ret.push_back(i);
      }
    }
    return ret;
  }
  
  void Node::increasePriority(void)
  {
    priority ++;
    for(unsigned int i = 0 ; i < elements.size() ; i++)
    {
      elements[i]->increasePriority();
    }
  }
  //-----------------------------------------------------------------------//
  
  Base::Base(void)
  {
    base_path_=ros::package::getPath("stdr_resources");
    base_node_ = new Node();
    base_node_->tag = "STDR_Parser_Root_Node";
  }
  
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
  
  void Base::printParsedXml(void)
  {
    ROS_ERROR("-----------------------------------------------------");
    printParsedXml(base_node_,"");
  }
  
  void Base::eliminateFilenames(void)
  {
    while(!eliminateFilenames(base_node_))
    {
    }
  }
  
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
  
  void Base::parse(std::string file_name)
  {
    // Must destroy prev tree
    parse(file_name,base_node_);
    
    parseMergableSpecifications();
    
    // Must do validity checks
    eliminateFilenames();
  }
  
  void Base::parse(std::string file_name,Node* n)
  {
    std::string path=base_path_ + std::string("/xmls/") + file_name;
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      ROS_ERROR("Failed to load file \"%s\"\n,%s", path.c_str(),doc.ErrorDesc());
    }
    parseLow(&doc,n);
  }
  
  void Base::parseMergableSpecifications(void)
  {
    std::string path=base_path_ + 
      std::string("/xmls/stdr_specific/stdr_multiple_allowed.xml");
    TiXmlDocument doc;
    bool loadOkay = doc.LoadFile(path.c_str());
    if (!loadOkay)
    {
      ROS_ERROR("Failed to load file \"%s\"\n,%s", path.c_str(),doc.ErrorDesc());
      return;
    }
    mergable_tags_ = explodeString(
      doc.FirstChild()->FirstChild()->Value(), ',');
  }
  
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
        else if(node_text!="allowed" && node_text!="required")
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
        else
        {
          ROS_ERROR("STDR XML parser Error : Specifications not in allowed \
or required : [%s] (%s) {%s}",base_tag.c_str(), base_type.c_str(), node_text.c_str());
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
  
  std::set<std::string> Base::explodeString(std::string s,char delimiter)
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
  
  void Base::printSpecifications(void)
  {
    for(std::map<std::string,ElSpecs>::iterator it = Specs::specs.begin();
      it != Specs::specs.end() ; it ++)
    {
      ROS_ERROR("[%s]",it->first.c_str());
      ROS_ERROR("  [allowed : ]",it->first.c_str());
      for(std::set<std::string>::iterator inner_it = 
        it->second.allowed.begin(); 
          inner_it != it->second.allowed.end() ; inner_it ++)
      {
        ROS_ERROR("    - %s",(*inner_it).c_str());
      }
      ROS_ERROR("  [required : ]",it->first.c_str());
      for(std::set<std::string>::iterator inner_it = 
        it->second.required.begin(); 
          inner_it != it->second.required.end() ; inner_it ++)
      {
        ROS_ERROR("    - %s",(*inner_it).c_str());
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"stdr_xml_parser");
  
  stdr_xml_parser::Base b;
  b.initialize();
  b.parse("simple_robot.xml");
  b.printParsedXml();
  //~ ros::spin();
  return 0;
}
