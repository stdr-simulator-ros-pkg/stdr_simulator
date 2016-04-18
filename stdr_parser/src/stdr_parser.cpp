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
  
  //!< Static initializations
  Node* Parser::base_node_ = new Node();
  
  /**
  @brief Default constructor
  @return void
  **/
  Parser::Parser(void)
  {
    
  }

  /**
  @brief Parses an xml file
  @param file_name [std::string] The xml filename
  @return void
  **/
  void Parser::parse(std::string file_name)
  {
    Parser::base_node_ = new Node();
    Parser::base_node_->tag = "STDR_Parser_Root_Node";
    
    // Must destroy prev tree
    try
    {
      if(file_name.find(".xml") != std::string::npos)
      {
        XmlParser::parse(file_name,base_node_);  
      }
      else if(file_name.find(".yaml") != std::string::npos)
      {
        YamlParser::parse(file_name,base_node_);
      }
      //~ base_node_->printParsedXml(base_node_,"");
      Validator::parseMergableSpecifications();
      
      while(!eliminateFilenames(base_node_));
      while(!mergeNodes(base_node_));
      mergeNodesValues(base_node_);
      
      Validator::validate(file_name, base_node_);
      
      //!< Uncomment to see the internal tree structure
      //~ base_node_->printParsedXml(base_node_,"");
    }
    catch(ParserException ex)
    {
      throw ex;
    }
    catch(YAML::ParserException& e)
    {
      std::string error = 
        std::string("Failed to load file '") + 
        file_name + std::string("'") +
        std::string("\nError was '") + std::string(e.what());
      
      delete base_node_;
      
      throw ParserException(error);
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

