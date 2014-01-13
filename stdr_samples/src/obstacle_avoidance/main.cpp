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
# include "stdr_samples/obstacle_avoidance/obstacle_avoidance.h"

/**
@brief The main node function
@param argc [int] Number of input arguments
@param argv [char] The input arguments
@return int : 0 for success
**/
int main(int argc,char **argv)
{
  ros::init(argc, argv, "stdr_obstacle_avoidance", ros::init_options::AnonymousName);
  stdr_samples::ObstacleAvoidance obj(argc, argv);
  ros::spin();
  return 0;
}
