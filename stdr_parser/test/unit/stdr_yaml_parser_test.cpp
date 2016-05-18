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
#include "stdr_parser/stdr_yaml_parser.h"

namespace stdr_parser
{

/**
 * @class YamlaParserTest
 * @brief Basic Test Fixture for testing YamlParser
 */
class YamlParserTest : public ::testing::Test
{
 protected:
  YamlParserTest()
  {
  }

  virtual void TearDown()
  {
    delete root_node_;
  }

  void init(const std::string& filename)
  {
    robot_file_ = ros::package::getPath("stdr_parser") +
                    filename;

    root_node_ = new Node();
    root_node_->tag = "STDR_Parser_Root_Node";
  }

  // Accessors for private methods of YamlParser
  void parseLow(const YAML::Node& node,Node* n)
  {
    YamlParser::parseLow(node, n);
  }
  
  //checks if two trees of Nodes are equal 
  void checkTree(Node* n1, Node* n2)
  {
    EXPECT_STREQ((n1)->tag.c_str(), (n2)->tag.c_str());
    EXPECT_STREQ((n1)->value.c_str(), (n2)->value.c_str());
    ASSERT_EQ(n1->elements.size(), n2->elements.size());
    for(std::vector<Node*>::iterator it1=(n1)->elements.begin(), it2=(n2)->elements.begin(); it1 != (n1)->elements.end(), it2!=(n2)->elements.end(); ++it1, ++it2)
    {
      checkTree(*it1,*it2);
    }
  }

  // Variables
  Node* root_node_;
  std::string robot_file_;

};

TEST_F(YamlParserTest, parseTestRobot1)
{
  init(std::string("/test/files/test_robot_1.yaml"));

  // parse the test file
  EXPECT_NO_THROW(YamlParser::parse(robot_file_, root_node_));

  EXPECT_EQ(root_node_->elements.size(), 1);


}

TEST_F(YamlParserTest, parseAlternateResourceLocation)
{
  init(std::string("/test/files/test_robot_2.yaml"));

  // parse the correct test file
  EXPECT_NO_THROW(YamlParser::parse(robot_file_, root_node_));


  TearDown();
  init(std::string("/test/files/test_robot_3.yaml"));

  // parse the incorrect test file
  EXPECT_THROW(YamlParser::parse(robot_file_, root_node_), ParserException);
}

TEST_F(YamlParserTest, parseLow)
{
    init(std::string("/test/files/test_robot_1.yaml"));
    std::string path = robot_file_;
    std::ifstream fin(path.c_str());
    
    if (!fin.good()) {
      throw ParserException("Failed to load '"+ robot_file_ +"', no such file!");
    }

#ifdef HAVE_NEW_YAMLCPP
    YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif

    root_node_->file_origin = robot_file_;
#ifndef HAVE_NEW_YAMLCPP
    root_node_->file_row = doc.GetMark().line;
#endif
    parseLow(doc,root_node_);

    //declare and fill the tree corresponding to file test_robot_1.yaml
    Node* test_node_=new Node();
    std::vector<Node*> child_node(77);
    for (unsigned i=0; i<child_node.size(); i++)
    {
      child_node.at(i)=new Node();
    }
    std::vector<Node*> test_children;
    child_node.at(0)->tag="STDR_Parser_Root_Node";
    child_node.at(1)->tag="robot";
    child_node.at(0)->elements.push_back(child_node.at(1));
    child_node.at(2)->tag = "filename";
    child_node.at(3)->tag = "robot_specifications";
    child_node.at(1)->elements.push_back(child_node.at(2));
    child_node.at(1)->elements.push_back(child_node.at(3));
    child_node.at(4)->tag = "footprint";
    child_node.at(5)->tag = "initial_pose";
    child_node.at(6)->tag = "laser";
    child_node.at(3)->elements.push_back(child_node.at(4));
    child_node.at(3)->elements.push_back(child_node.at(5));
    child_node.at(3)->elements.push_back(child_node.at(6));
    child_node.at(7)->tag = "footprint_specifications";
    child_node.at(4)->elements.push_back(child_node.at(7));
    child_node.at(8)->tag = "radius";
    child_node.at(57)->value="0.05";
    child_node.at(8)->elements.push_back(child_node.at(57));
    child_node.at(7)->elements.push_back(child_node.at(8));
    child_node.at(9)->tag = "x";
    child_node.at(37)->value = "3";
    child_node.at(9)->elements.push_back(child_node.at(37));
    child_node.at(10)->tag = "y";
    child_node.at(38)->value = "2";
    child_node.at(10)->elements.push_back(child_node.at(38));
    child_node.at(11)->tag = "theta";
    child_node.at(39)->value = "1.57";
    child_node.at(11)->elements.push_back(child_node.at(39));
    child_node.at(5)->elements.push_back(child_node.at(9));
    child_node.at(5)->elements.push_back(child_node.at(10));
    child_node.at(5)->elements.push_back(child_node.at(11));
    child_node.at(12)->tag = "filename";
    child_node.at(13)->tag = "laser_specifications";
    child_node.at(6)->elements.push_back(child_node.at(12));
    child_node.at(6)->elements.push_back(child_node.at(13));
    child_node.at(14)->tag = "pose";
    child_node.at(13)->elements.push_back(child_node.at(14));
    child_node.at(15)->tag = "theta";
    child_node.at(40)->value = "-3.1415";
    child_node.at(15)->elements.push_back(child_node.at(40));
    child_node.at(14)->elements.push_back(child_node.at(15));
    child_node.at(16)->tag = "robot";
    child_node.at(2)->elements.push_back(child_node.at(16));
    child_node.at(17)->tag = "robot_specifications";
    child_node.at(16)->elements.push_back(child_node.at(17));
    child_node.at(18)->tag = "initial_pose";
    child_node.at(17)->elements.push_back(child_node.at(18));
    child_node.at(19)->tag = "x";
    child_node.at(41)->value = "0";
    child_node.at(19)->elements.push_back(child_node.at(41));
    child_node.at(20)->tag = "y";
    child_node.at(42)->value = "0";
    child_node.at(20)->elements.push_back(child_node.at(42));
    child_node.at(21)->tag = "theta";
    child_node.at(43)->value = "0";
    child_node.at(21)->elements.push_back(child_node.at(43));
    child_node.at(18)->elements.push_back(child_node.at(19));
    child_node.at(18)->elements.push_back(child_node.at(20));
    child_node.at(18)->elements.push_back(child_node.at(21));
    child_node.at(22)->tag = "laser";
    child_node.at(12)->elements.push_back(child_node.at(22));
    child_node.at(23)->tag = "laser_specifications";
    child_node.at(22)->elements.push_back(child_node.at(23));
    child_node.at(24)->tag = "max_angle";
    child_node.at(44)->value = "2.09439510239";
    child_node.at(24)->elements.push_back(child_node.at(44));
    child_node.at(25)->tag = "min_angle";
    child_node.at(45)->value = "-2.09439510239";
    child_node.at(25)->elements.push_back(child_node.at(45));
    child_node.at(26)->tag = "max_range";
    child_node.at(46)->value = "4.09";
    child_node.at(26)->elements.push_back(child_node.at(46));
    child_node.at(27)->tag = "min_range";
    child_node.at(47)->value = "0.06";
    child_node.at(27)->elements.push_back(child_node.at(47));
    child_node.at(28)->tag = "num_rays";
    child_node.at(48)->value = "667";
    child_node.at(28)->elements.push_back(child_node.at(48));
    child_node.at(29)->tag = "frequency";
    child_node.at(49)->value = "10";
    child_node.at(29)->elements.push_back(child_node.at(49));
    child_node.at(23)->elements.push_back(child_node.at(24));
    child_node.at(23)->elements.push_back(child_node.at(25));
    child_node.at(23)->elements.push_back(child_node.at(26));
    child_node.at(23)->elements.push_back(child_node.at(27));
    child_node.at(23)->elements.push_back(child_node.at(28));
    child_node.at(23)->elements.push_back(child_node.at(29));
    child_node.at(59)->tag = "pose";
    child_node.at(23)->elements.push_back(child_node.at(59));
    child_node.at(65)->tag = "x";
    child_node.at(50)->value = "0";
    child_node.at(65)->elements.push_back(child_node.at(50));
    child_node.at(66)->tag = "y";
    child_node.at(51)->value = "0";
    child_node.at(66)->elements.push_back(child_node.at(51));
    child_node.at(67)->tag = "theta";
    child_node.at(52)->value = "0";
    child_node.at(67)->elements.push_back(child_node.at(52));
    child_node.at(59)->elements.push_back(child_node.at(65));
    child_node.at(59)->elements.push_back(child_node.at(66));
    child_node.at(59)->elements.push_back(child_node.at(67));
    child_node.at(68)->tag = "noise";
    child_node.at(23)->elements.push_back(child_node.at(68));
    child_node.at(69)->tag = "filename";
    child_node.at(68)->elements.push_back(child_node.at(69));
    child_node.at(70)->tag = "noise_specifications";
    child_node.at(68)->elements.push_back(child_node.at(70));
    child_node.at(71)->tag = "noise_mean";
    child_node.at(53)->value = "0.5";
    child_node.at(71)->elements.push_back(child_node.at(53));
    child_node.at(72)->tag = "noise_std";
    child_node.at(54)->value = "0.05";
    child_node.at(72)->elements.push_back(child_node.at(54));
    child_node.at(70)->elements.push_back(child_node.at(71));
    child_node.at(70)->elements.push_back(child_node.at(72));
    child_node.at(73)->tag = "noise";
    child_node.at(69)->elements.push_back(child_node.at(73));
    child_node.at(74)->tag = "noise_specifications";
    child_node.at(73)->elements.push_back(child_node.at(74));
    child_node.at(75)->tag = "noise_mean";
    child_node.at(55)->value = "0.1";
    child_node.at(75)->elements.push_back(child_node.at(55));
    child_node.at(76)->tag = "noise_std";
    child_node.at(56)->value = "0.01";
    child_node.at(76)->elements.push_back(child_node.at(56));
    child_node.at(74)->elements.push_back(child_node.at(75));
    child_node.at(74)->elements.push_back(child_node.at(76));
    checkTree(child_node.at(0), root_node_);
  
}



}  // namespace stdr_parser
