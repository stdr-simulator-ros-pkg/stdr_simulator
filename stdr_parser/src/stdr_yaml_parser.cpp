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

#include "stdr_parser/stdr_yaml_parser.h"

namespace stdr_parser
{
  /**
  @brief Default constructor
  @return void
  **/
  YamlParser::YamlParser(void)
  {

  }
  
  /**
  @brief Parses an xml file
  @param file_name [std::string] The xml filename
  @return void
  **/
  void YamlParser::parse(std::string file_name, Node* base_node)
  {
    std::string path=ros::package::getPath("stdr_resources") + 
      std::string("/yamls/") + file_name;
    std::ifstream fin(path.c_str());
    #ifdef HAVE_NEW_YAMLCPP
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
    
    base_node->file_origin = file_name;
    base_node->file_row = doc.GetMark().line;
    
    parseLow(doc,base_node);

    try
    {
      while(!eliminateFilenames(base_node));
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    
    while(!mergeNodes(base_node));

    mergeNodesValues(base_node);
  }
  
  /**
  @brief Low-level recursive function for parsing the yaml file
  @param node [YAML::Node&] The yaml node to start from
  @param n [Node*] The stdr tree node to update
  @return void
  **/
  void YamlParser::parseLow(const YAML::Node& node,Node* n)
  {
    if(node.Type() == YAML::NodeType::Scalar)
    {
      Node* new_node = new Node();
      std::string s;
      node >> s;
      new_node->value = s;
      new_node->file_origin = n->file_origin;
      new_node->file_row = node.GetMark().line;
      n->elements.push_back(new_node);
    }
    else if(node.Type() == YAML::NodeType::Sequence)
    {
      for(unsigned int i = 0 ; i < node.size() ; i++) 
      {
        parseLow(node[i],n);
      }
    }
    else if(node.Type() == YAML::NodeType::Map)
    {
      std::string s;
      for(YAML::Iterator it = node.begin() ; it != node.end() ; it++) 
      {
        Node* new_node = new Node();
        it.first() >> s;
        new_node->tag = s;
        new_node->file_origin = n->file_origin;
        new_node->file_row = node.GetMark().line;
        n->elements.push_back(new_node);
        if(s == "filename")
        {
          std::string file_name;
          it.second() >> file_name;
          std::string path=ros::package::getPath("stdr_resources") + 
            std::string("/yamls/") + file_name;
          std::ifstream fin(path.c_str());
          #ifdef HAVE_NEW_YAMLCPP
            YAML::Node doc = YAML::Load(fin);
          #else
            YAML::Parser parser(fin);
            YAML::Node doc;
            parser.GetNextDocument(doc);
          #endif
          new_node->file_origin = file_name;
          new_node->file_row = doc.GetMark().line;
          
          parseLow(doc,new_node);
        }
        else
        {
          parseLow(it.second(),new_node);
        }
      }
    }
  }
  
 
  /**
  @brief Recursive function - Expands the 'filename' nodes and eliminates them
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is a 'filename' node was expanded
  **/
  bool YamlParser::eliminateFilenames(Node* n)
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
  void YamlParser::mergeNodesValues(Node* n)
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
  @brief Recursive function - Merges the nodes that do not exist in non_mergable_tags_
  @param n [Node*] The stdr xml tree node to begin
  @return bool : True is a ndoe was merged
  **/
  bool YamlParser::mergeNodes(Node* n)
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
      if(Specs::non_mergable_tags.find(tag) == Specs::non_mergable_tags.end())
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
}

