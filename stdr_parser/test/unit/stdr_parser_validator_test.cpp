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
   * Christos Zalidis, zalidis@gmail.com
******************************************************************************/

#include <gtest/gtest.h>
#include <iterator> 
#include <tinyxml.h>
#include "stdr_parser/stdr_parser_tools.h"
#include "stdr_parser/stdr_parser_validator.h"
#include "stdr_parser/stdr_parser_node.h"
#include <typeinfo>
#include <vector>
namespace stdr_parser
{


/**
 * @class ValidatorTest
 * @brief Basic Test Fixture for testing Validator
 */
class ValidatorTest : public ::testing::Test
{
 protected:
  ValidatorTest()
  {
  }

  virtual void TearDown()
  {
    
  }

  void init(const std::string& filename)
  {
    specs_file_ = ros::package::getPath("stdr_parser") +
                    filename;
  }
   
  // Accessors for private methods of XmlParser
  std::map<std::string,ElSpecs> parseSpecifications(TiXmlNode* node)
  {
    return Validator::parseSpecifications(node);
  }

  void validityAllowedCheck(std::string file_name, Node* n)
  {
    Validator::validityAllowedCheck(file_name,n);
  }
  
  void validityRequiredCheck(std::string file_name, Node* n)
  {
    Validator::validityRequiredCheck(file_name,n);
  }

  void validate(std::string file_name, Node* n)
  {
    Validator::validate(file_name,n);
  }

  // Variables
  std::string specs_file_;
  Node* dummy_node;

};

TEST_F(ValidatorTest, parseMergableSpecificationsTestThrow)
{
  init(std::string("/test/files/stdr_multiple_allowed.xml"));
  //parse stdr_multiple_allowed.xml
  EXPECT_NO_THROW(Validator::parseMergableSpecifications(specs_file_));

}



TEST_F(ValidatorTest, parseMergableSpecificationsRightTags)
{
  init(std::string("/test/files/stdr_multiple_allowed.xml"));
  //parse stdr_multiple_allowed.xml to get non_mergable_tags
  std::set<std::string> non_mergable_tags_or = Validator::parseMergableSpecifications(specs_file_);
  //fill non_mergable_tags_test as defined in stdr_multiple_allowed.xml
  std::string tags[] = {"robot","laser","sonar","rfid_reader","point","co2_sensor","thermal_sensor","sound_sensor"};
  std::set<std::string> non_mergable_tags_test(tags, tags + sizeof(tags) / sizeof(tags[0]));
  for (std::set<std::string>::iterator it1 = non_mergable_tags_test.begin(), it2 = non_mergable_tags_or.begin(); it1 != non_mergable_tags_test.end() && it2 != non_mergable_tags_or.end(); ++it1, ++it2)
  {
    std::string test = *it1; 
    std::string original = *it2;
    EXPECT_STREQ(test.c_str(),original.c_str());
  }
  
} 

TEST_F(ValidatorTest,parseSpecificationsTestThrow)
{
  Validator::clearSpecs();
  init(std::string("/test/files/stdr_specifications.xml"));
  std::string path = extractDirname(specs_file_);
  TiXmlDocument test_doc;
  bool loadOkay = test_doc.LoadFile(specs_file_.c_str());
  if (!loadOkay)
  {
    std::string error =    
    std::string("Failed to load specifications file.\nShould be at '") + 
    path + std::string("'\nError was") + std::string(test_doc.ErrorDesc());
    throw ParserException(error);
  }
  //parse stdr_speficications.xml
  EXPECT_NO_THROW(parseSpecifications(&test_doc));
}


TEST_F(ValidatorTest,parseSpecificationsRightSpecs)
{
  Validator::clearSpecs();
  //fill map specifications as defined in stdr_specifications.xml
  init(std::string("/test/files/stdr_specifications.xml"));
  char* tags_array[] = {"environment", "map", "map_specifications","origin","robot","robot_specifications",
  "initial_pose","pose","footprint","footprint_specifications","points","point","laser","laser_specifications"
  ,"noise","noise_specifications","sonar","sonar_specifications","rfid_reader",
  "rfid_reader_specifications","co2_sensor","co2_sensor_specifications",
  "thermal_sensor","thermal_sensor_specifications","sound_sensor","sound_sensor_specifications",
  "kinematic","kinematic_specifications","kinematic_parameters","rfid_tag","x","y","theta"
  ,"max_angle","min_angle","max_range","min_range","num_rays","frequency","frame_id",
  "noise_mean","noise_std","angle_span","signal_cutoff","radius","cone_angle",
  "kinematic_model","a_ux_ux","a_ux_uy","a_ux_w","a_uy_ux","a_uy_uy",
  "a_uy_w","a_w_ux","a_w_uy","a_w_w","a_g_ux","a_g_uy","a_g_w"
  };
 
  std::vector<std::string> tags(tags_array, tags_array + 60);
  std::map<std::string,ElSpecs> specifications;
  for(std::vector<std::string>::const_iterator it_tags = tags.begin(); it_tags != tags.end(); ++it_tags)
  {
    ElSpecs elspecs_test;
    elspecs_test.default_value=""; 
    std::vector<std::string> tmp_allowed_vect;
    std::vector<std::string> tmp_required_vect;
    if(*it_tags=="environment")
    {
      std::string tmp_allowed_array[] = {"map", "robot", "rfid_tag"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 3 ) ; 
      std::string tmp_required_array[] = {"map"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
      
    }
    if(*it_tags=="map")
    {
      std::string tmp_allowed_array[] = {"filename","map_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"map_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="map_specifications")
    {
      std::string tmp_allowed_array[] = {"image","resolution","origin","free_thresh","negate","occupied_thresh"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 6 ) ; 
      std::string tmp_required_array[] = {"image","resolution"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 2 ) ;
    }
    if(*it_tags=="robot")
    {
      std::string tmp_allowed_array[] = {"filename","robot_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"robot_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="robot_specifications")
    {
      std::string tmp_allowed_array[] = {"initial_pose","footprint","laser","sonar","rfid_reader","kinematic"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 6 ) ; 
    }
    if(*it_tags=="initial_pose" || *it_tags=="pose" || *it_tags=="origin")
    {
      std::string tmp_allowed_array[] = {"x","y","theta"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 3 ) ; 
    }
    if(*it_tags=="footprint")
    {
      std::string tmp_allowed_array[] = {"filename","footprint_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"footprint_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="footprint_specifications")
    {
      std::string tmp_allowed_array[] = {"radius","points"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
    }
    if(*it_tags=="points")
    {
      std::string tmp_allowed_array[] = {"point"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 1 ) ; 
    }
    if(*it_tags=="point")
    {
      std::string tmp_allowed_array[] = {"x","y"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"x","y"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 2 ) ;
    }
    if(*it_tags=="laser")
    {
      std::string tmp_allowed_array[] = {"filename","laser_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"laser_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="laser_specifications")
    {
      std::string tmp_allowed_array[] = {"max_angle","min_angle","max_range","min_range","num_rays","frequency","frame_id","pose","noise"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 9 ) ; 
    }
    if(*it_tags=="noise")
    {
      std::string tmp_allowed_array[] = {"filename","noise_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"noise_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="noise_specifications")
    {
      std::string tmp_allowed_array[] = {"noise_mean","noise_std"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
    }
    if(*it_tags=="sonar")
    {
      std::string tmp_allowed_array[] = {"filename","sonar_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"sonar_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="sonar_specifications")
    {
      std::string tmp_allowed_array[] = {"cone_angle","max_range","min_range","frequency","frame_id","pose","noise"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 7 ) ; 
    }
    if(*it_tags=="co2_sensor")
    {
      std::string tmp_allowed_array[] = {"filename","co2_sensor_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"co2_sensor_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="co2_sensor_specifications")
    {
      std::string tmp_allowed_array[] = {"max_range","frequency","frame_id","pose"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 4 ) ; 
    }
    if(*it_tags=="thermal_sensor")
    {
      std::string tmp_allowed_array[] = {"filename","thermal_sensor_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"thermal_sensor_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="thermal_sensor_specifications" || *it_tags=="sound_sensor_specifications")
    {
      std::string tmp_allowed_array[] = {"max_range","angle_span","frequency","frame_id","pose"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 5 ) ; 
    }
    if(*it_tags=="sound_sensor")
    {
      std::string tmp_allowed_array[] = {"filename","sound_sensor_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"sound_sensor_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="kinematic")
    {
      std::string tmp_allowed_array[] = {"filename","kinematic_specifications"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"kinematic_specifications"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ;
    }
    if(*it_tags=="kinematic_specifications")
    {
     std::string tmp_allowed_array[] = {"kinematic_model","kinematic_parameters"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 2 ) ; 
      std::string tmp_required_array[] = {"kinematic_model"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 1 ) ; 
   
    }
    if(*it_tags=="kinematic_parameters")
    {
      std::string tmp_allowed_array[] =   {"a_ux_ux","a_ux_uy","a_ux_w","a_uy_ux","a_uy_uy","a_uy_w","a_w_ux","a_w_uy","a_w_w","a_g_ux","a_g_uy","a_g_w"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 12 ) ; 
      std::string tmp_required_array[] = {"a_ux_ux","a_ux_uy","a_ux_w","a_uy_ux","a_uy_uy","a_uy_w","a_w_ux","a_w_uy","a_w_w","a_g_ux","a_g_uy","a_g_w"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 12 ) ;
    }
    if(*it_tags=="rfid_tag")
    {
      std::string tmp_allowed_array[] = {"x","y","message"};
      tmp_allowed_vect.insert(tmp_allowed_vect.begin() , tmp_allowed_array , tmp_allowed_array + 3 ) ; 
      std::string tmp_required_array[] = {"x","y"};
      tmp_required_vect.insert(tmp_required_vect.begin() , tmp_required_array , tmp_required_array + 2 ) ;
    }
    if(*it_tags=="x" || *it_tags=="y" || *it_tags=="theta" || *it_tags=="min_range" || *it_tags=="noise_mean" || *it_tags=="noise_std"  || *it_tags=="a_ux_ux" || *it_tags=="a_ux_uy" || *it_tags=="a_ux_w" || *it_tags=="a_uy_ux" || *it_tags=="a_uy_uy" || *it_tags=="a_uy_w" || *it_tags=="a_w_ux" || *it_tags=="a_w_uy" || *it_tags=="a_w_w" || *it_tags=="a_g_ux" || *it_tags=="a_g_uy" || *it_tags=="a_g_w")
    {
       elspecs_test.default_value="0";
    }
    if (*it_tags=="signal_cutoff")
    {
      elspecs_test.default_value = "0.0";
     }
    if(*it_tags=="max_angle")
    {
       elspecs_test.default_value = "1.57";
    }
     if(*it_tags=="min_angle")
    {
       elspecs_test.default_value = "-1.57";
    }
    if(*it_tags=="max_range")
    {
       elspecs_test.default_value = "4";
    }
    if(*it_tags=="num_rays")
    {
       elspecs_test.default_value = "180";
    }
    if(*it_tags=="frequency")
    {
       elspecs_test.default_value = "10";
    }
    if(*it_tags=="angle_span")
    {
       elspecs_test.default_value = "360.0";
    }
    if(*it_tags=="radius")
    {
       elspecs_test.default_value = "0.3";
    }
    if(*it_tags=="cone_angle")
    {
       elspecs_test.default_value = "1.0";
    }
    if(*it_tags=="kinematic_model")
    {
       elspecs_test.default_value = "ideal";
    }
   
    for(std::vector<std::string>::const_iterator it_allowed = tmp_allowed_vect.begin(); it_allowed != tmp_allowed_vect.end(); ++it_allowed)
    {
      elspecs_test.allowed.insert(*it_allowed);
    }
    for(std::vector<std::string>::const_iterator it_required = tmp_required_vect.begin(); it_required != tmp_required_vect.end(); ++it_required)
    {
     
      elspecs_test.required.insert(*it_required);
    }
    specifications.insert(std::pair<std::string,ElSpecs>(*it_tags,elspecs_test));
   
  }
  std::string path=extractDirname(specs_file_);
  TiXmlDocument test_doc;
  bool loadOkay = test_doc.LoadFile(specs_file_.c_str());
  if (!loadOkay)
  {
    std::string error =    
    std::string("Failed to load specifications file.\nShould be at '") + 
    path + std::string("'\nError was") + std::string(test_doc.ErrorDesc());
    throw ParserException(error);
  }
  //parse specifications.xml to get elspecs
  std::map<std::string,ElSpecs> map_or = parseSpecifications(&test_doc);

  //compare required, allowed and default_value tags
  for(std::map<std::string,ElSpecs>::const_iterator it = map_or.begin();
  it != map_or.end(); ++it)
  {
    ElSpecs elspecs_test = specifications[it->first.c_str()];
    ElSpecs elspecs_or = it->second;
    std::set<std::string> required_tags_or = elspecs_or.required;
    for (std::set<std::string>::iterator it1 = required_tags_or.begin(), it1_test=elspecs_test.required.begin(); it1 != required_tags_or.end() && it1_test != elspecs_test.required.end() ; ++it1, ++it1_test)
    {
      std::string required_tag_or = *it1; 
      std::string required_tag_test = *it1_test;
      EXPECT_STREQ(required_tag_or.c_str(),required_tag_test.c_str());
    }
    std::set<std::string> allowed_tags_or = elspecs_or.allowed;
    for (std::set<std::string>::iterator it2 = allowed_tags_or.begin(), it2_test=elspecs_test.allowed.begin(); it2 != allowed_tags_or.end() && it2_test != elspecs_test.allowed.end() ; ++it2, ++it2_test)
    {
      std::string allowed_tag_or = *it2; 
      std::string allowed_tag_test = *it2_test; 
      EXPECT_STREQ(allowed_tag_or.c_str(),allowed_tag_test.c_str());
    }
    std::string default_value_or = elspecs_or.default_value;
    EXPECT_STREQ(default_value_or.c_str(),elspecs_test.default_value.c_str());
  }
}

TEST_F(ValidatorTest,validityAllowedCheckValueNode)
{
  Node* dummy_node = new Node();
  dummy_node->value = "5";
  std::string file_name = "";
  //check if value node passes validityAllowedCheck
  EXPECT_NO_THROW(validityAllowedCheck(file_name,dummy_node));
}

TEST_F(ValidatorTest,validityAllowedCheckValidTagNode)
{
  Node* dummy_node = new Node();
  dummy_node->tag = "sonar_specifications";
  std::string tags[] = {"cone_angle","max_range","min_range","frequency","frame_id","pose","noise"};
  std::set<std::string> tags_set(tags, tags + sizeof(tags) / sizeof(tags[0]));
  for(std::set<std::string>::const_iterator it = tags_set.begin(); it != tags_set.end(); ++it)
  {
    Node* new_node=new Node();
    new_node->tag=*it;
    dummy_node->elements.push_back(new_node);
  }
  std::string file_name="";
  //check if tag node passes validityAllowedCheck
  EXPECT_NO_THROW(validityAllowedCheck(file_name,dummy_node));
}

TEST_F(ValidatorTest,validityAllowedCheckInvalidTagNode)
{
  Node* dummy_node = new Node();
  dummy_node->tag = "sonar_specifications";
  std::string tags[] = {"cone_angle","max_range","min_range","frequency","num_rays"};
  std::set<std::string> tags_set(tags, tags + sizeof(tags) / sizeof(tags[0]));
  for(std::set<std::string>::const_iterator it = tags_set.begin(); it != tags_set.end(); ++it)
  {
    Node* new_node = new Node();
    new_node->tag =*it;
    dummy_node->elements.push_back(new_node);
  }
  std::string file_name = "";
  //check if tag node, whose elements include not allowed tags, fails to pass validityAllowedCheck
  EXPECT_THROW(validityAllowedCheck(file_name,dummy_node),ParserException);
}

TEST_F(ValidatorTest,validityAllowedCheckInvalidTagNodeRec)
{
  Node* dummy_node = new Node();
  dummy_node->tag = "environment";
  std::string tags[] = {"map","robot","rfid_tag"};
  std::set<std::string> tags_set(tags, tags + sizeof(tags) / sizeof(tags[0]));
  for(std::set<std::string>::const_iterator it = tags_set.begin(); it != tags_set.end(); ++it)
  {
    Node* new_node = new Node();
    new_node->tag = *it;
    dummy_node->elements.push_back(new_node);
  }
  Node* new_node = new Node();
  new_node->tag = "image";
  dummy_node->elements.at(0)->elements.push_back(new_node);
  std::string file_name = "";
  //check if tag node with one element, whose elements include not allowed tags, fails to pass validityAllowedCheck
  EXPECT_THROW(validityAllowedCheck(file_name,dummy_node),ParserException);
}

TEST_F(ValidatorTest,validityRequiredCheckValueNode)
{
  Node* dummy_node = new Node();
  dummy_node->value = "5";
  std::string file_name = "";
  //check if value node passes validityRequiredCheck
  EXPECT_NO_THROW(validityRequiredCheck(file_name,dummy_node));
}

TEST_F(ValidatorTest,validityRequiredCheckValidTagNode)
{
  Node* dummy_node = new Node();
  dummy_node->tag = "kinematic_parameters";
  std::string tags[] = {"a_ux_ux","a_ux_uy","a_ux_w","a_uy_ux","a_uy_uy","a_uy_w","a_w_ux","a_w_uy","a_w_w","a_g_ux","a_g_uy","a_g_w"};
  std::set<std::string> tags_set(tags, tags + sizeof(tags) / sizeof(tags[0]));
  for(std::set<std::string>::const_iterator it = tags_set.begin(); it != tags_set.end(); ++it)
  {
    Node* new_node = new Node();
    new_node->tag = *it;
    dummy_node->elements.push_back(new_node);
  }
  std::string file_name = "";
  //check if tag node passes validityRequiredCheck
  EXPECT_NO_THROW(validityRequiredCheck(file_name,dummy_node));
}

TEST_F(ValidatorTest,validityRequiredCheckInvalidTagNode)
{
  Node* dummy_node = new Node();
  dummy_node->tag = "map_specifications";
  std::string tags[] = {"image"};
  std::set<std::string> tags_set(tags, tags + sizeof(tags) / sizeof(tags[0]));
  for(std::set<std::string>::const_iterator it = tags_set.begin(); it != tags_set.end(); ++it)
  {
    Node* new_node = new Node();
    new_node->tag = *it;
    dummy_node->elements.push_back(new_node);
  }
  std::string file_name = "";
  //check if tag node, whose elements don't include all required tags, fails to pass validityRequiredCheck
  EXPECT_THROW(validityRequiredCheck(file_name,dummy_node),ParserException);
}

TEST_F(ValidatorTest,validateValidTagNode)
{
  Node* dummy_node = new Node();
  dummy_node->tag = "point";
  std::string tags[] = {"x","y"};
  std::set<std::string> tags_set(tags, tags + sizeof(tags) / sizeof(tags[0]));
  std::set<std::string>::const_iterator it;
  for(it = tags_set.begin(); it != tags_set.end(); ++it)
  {
    Node* new_node = new Node();
    new_node->tag = *it;
    dummy_node->elements.push_back(new_node);
  }
  std::string file_name = "";
  //check if tag node passes validate
  EXPECT_NO_THROW(validate(file_name,dummy_node));
}

TEST_F(ValidatorTest,clearSpecsIsEmpty)
{
  Specs specs;
  Specs returned = Validator::clearSpecs();
  EXPECT_EQ(specs,returned);
}

}  // namespace stdr_parser
