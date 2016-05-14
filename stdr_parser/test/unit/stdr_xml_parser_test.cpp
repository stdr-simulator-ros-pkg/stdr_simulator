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
#include "stdr_parser/stdr_xml_parser.h"

namespace stdr_parser
{

/**
 * @class XmlParserTest
 * @brief Basic Test Fixture for testing XmlParser
 */
class XmlParserTest : public ::testing::Test
{
 protected:
  XmlParserTest()
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

  // Accessors for private methods of XmlParser
  void parseLow(TiXmlNode* node, Node* n)
  {
    XmlParser::parseLow(node, n);
  }

  // Variables
  Node* root_node_;
  std::string robot_file_;

};

TEST_F(XmlParserTest, parseTestRobot1)
{
  init(std::string("/test/files/test_robot_1.xml"));

  // parse the test file
  EXPECT_NO_THROW(XmlParser::parse(robot_file_, root_node_));

  EXPECT_EQ(root_node_->elements.size(), 1);

  //root_node->printParsedXml(root_node, "--");
}

TEST_F(XmlParserTest, parseAlternateResourceLocation)
{
  init(std::string("/test/files/test_robot_2.xml"));

  // parse the correct test file
  EXPECT_NO_THROW(XmlParser::parse(robot_file_, root_node_));


  TearDown();
  init(std::string("/test/files/test_robot_3.xml"));

  // parse the incorrect test file
  EXPECT_THROW(XmlParser::parse(robot_file_, root_node_), ParserException);
}

/*TEST_F(XmlParserTest, parseLow)
{
  //TODO
}*/

}  // namespace stdr_parser
