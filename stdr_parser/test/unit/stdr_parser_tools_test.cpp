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
#include "stdr_parser/stdr_parser_node.h"

namespace stdr_parser
{

TEST(ToolsTest, extractFilename)
{
  std::string s1 = "/home/user/robot.yaml";
  std::string s2 = "/home/user/robot.xml";
  std::string s3 = "/home/user/robot";

  EXPECT_STREQ(extractFilename(s1).c_str(), "robot.yaml");
  EXPECT_STREQ(extractFilename(s2).c_str(), "robot.xml");
  EXPECT_STREQ(extractFilename(s3).c_str(), "robot");
}

TEST(ToolsTest, extractDirname)
{
  std::string s1 = "/home/user/robot.yaml";
  std::string s2 = "/home/user/robot.xml";
  std::string s3 = "/home/user/robot";

  EXPECT_STREQ(extractDirname(s1).c_str(), "/home/user");
  EXPECT_STREQ(extractDirname(s2).c_str(), "/home/user");
  EXPECT_STREQ(extractDirname(s3).c_str(), "/home/user");
}

}  // namespace stdr_parser
