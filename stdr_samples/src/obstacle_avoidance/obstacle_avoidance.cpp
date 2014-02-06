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
@namespace stdr_samples
@brief The main namespace for STDR Samples
**/ 
namespace stdr_samples
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  ObstacleAvoidance::ObstacleAvoidance(int argc,char **argv)
  {
    if(argc != 3)
    {
      ROS_ERROR(
        "Usage : stdr_obstacle avoidance <robot_frame_id> <laser_frame_id>");
      exit(0);
    }
    laser_topic_ = std::string("/") +
      std::string(argv[1]) + std::string("/") + std::string(argv[2]);
    speeds_topic_ = std::string("/") +
      std::string(argv[1]) + std::string("/cmd_vel");
      
    subscriber_ = n_.subscribe(
      laser_topic_.c_str(), 
      1, 
      &ObstacleAvoidance::callback,
      this);
      
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  ObstacleAvoidance::~ObstacleAvoidance(void)
  {
    
  }
  
  /**
  @brief Callback for the ros laser message
  @param msg [const sensor_msgs::LaserScan&] The new laser scan message
  @return void
  **/
  void ObstacleAvoidance::callback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;
    float linear = 0, rotational = 0;
    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i];
      linear -= cos(scan_.angle_min + i * scan_.angle_increment) 
        / (1.0 + real_dist * real_dist);
      rotational -= sin(scan_.angle_min + i * scan_.angle_increment) 
        / (1.0 + real_dist * real_dist);
    }
    geometry_msgs::Twist cmd;
    
    linear /= scan_.ranges.size();
    rotational /= scan_.ranges.size();
    
    //~ ROS_ERROR("%f %f",linear,rotational);
    
    if(linear > 0.3)
    {
      linear = 0.3;
    }
    else if(linear < -0.3)
    {
      linear = -0.3;
    }

    cmd.linear.x = 0.3 + linear;
    cmd.angular.z = rotational;
    cmd_vel_pub_.publish(cmd);
  }
}
