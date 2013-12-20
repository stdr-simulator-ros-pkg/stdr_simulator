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

#include "stdr_gui/stdr_tools.h"

namespace stdr_gui_tools
{

  std::string getRosPackagePath(std::string package)
  {
    return ros::package::getPath(package.c_str());
  }
  
  QString getLiteralTime(int ms)
  {
    QString str;
    int h = ms / (1000 * 60 * 60);
    int m = ms / (1000 * 60) - h * 60;
    int s = ms / 1000 - h * 60 * 60 - m * 60;
    int ms_ = ms - h * 60 * 60 * 1000 - m * 1000 * 60 - s * 1000;
    if(h)
    {
      str += QString().setNum(h) + QString(" h ");
    }
    if(m || h)
    {
      str += QString().setNum(m) + QString(" min ");
    }
    if(s || h || m)
    {
      str += QString().setNum(s) + QString(" sec ");
    }
    if(ms_ || s || h || m)
    {
      str += QString().setNum(ms_) + QString(" ms");
    }
    return str;
  }
  
  void printSonarMsg(stdr_msgs::SonarSensorMsg &msg)
  {
    ROS_ERROR("Sonar sensor msg :");
    ROS_ERROR("\tMax range : %f",msg.maxRange);
    ROS_ERROR("\tMin range : %f",msg.minRange);
    ROS_ERROR("\tCone angle : %f",msg.coneAngle);
    ROS_ERROR("\tFrequency : %f",msg.frequency);
    ROS_ERROR("\tNoise :");
    ROS_ERROR("\t\tMean : %f",msg.noise.noiseMean);
    ROS_ERROR("\t\tStd : %f",msg.noise.noiseStd);
    ROS_ERROR("\tFrame id : %s",msg.frame_id.c_str());
    ROS_ERROR("\tRelative pose :");
    ROS_ERROR("\t\tx : %f",msg.pose.x);
    ROS_ERROR("\t\ty : %f",msg.pose.y);
    ROS_ERROR("\t\ttheta : %f",msg.pose.theta);
  }
  
  void printPose2D(geometry_msgs::Pose2D &msg)
  {
    ROS_ERROR("Pose 2D :");
    ROS_ERROR("\tx : %f",msg.x);
    ROS_ERROR("\ty : %f",msg.y);
    ROS_ERROR("\ttheta : %f",msg.theta);
  }
  
  stdr_msgs::RobotMsg fixRobotAnglesToRad(stdr_msgs::RobotMsg rmsg)
  {
    rmsg.initialPose.theta = 
      rmsg.initialPose.theta / 180.0 * STDR_PI;
    for(unsigned int i = 0 ; i < rmsg.laserSensors.size() ; i++)
    {
      rmsg.laserSensors[i].maxAngle = 
        rmsg.laserSensors[i].maxAngle / 180.0 * STDR_PI;
      rmsg.laserSensors[i].minAngle = 
        rmsg.laserSensors[i].minAngle / 180.0 * STDR_PI;
      rmsg.laserSensors[i].pose.theta = 
        rmsg.laserSensors[i].pose.theta / 180.0 * STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.sonarSensors.size() ; i++)
    {
      rmsg.sonarSensors[i].coneAngle = 
        rmsg.sonarSensors[i].coneAngle / 180.0 * STDR_PI;
      rmsg.sonarSensors[i].pose.theta = 
        rmsg.sonarSensors[i].pose.theta / 180.0 * STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.rfidSensors.size() ; i++)
    {
      rmsg.rfidSensors[i].angleSpan = 
        rmsg.rfidSensors[i].angleSpan / 180.0 * STDR_PI;
      rmsg.rfidSensors[i].pose.theta = 
        rmsg.rfidSensors[i].pose.theta / 180.0 * STDR_PI;
    }
    return rmsg;
  }
  
  stdr_msgs::RobotMsg fixRobotAnglesToDegrees(stdr_msgs::RobotMsg rmsg)
  {
    rmsg.initialPose.theta = 
      rmsg.initialPose.theta * 180.0 / STDR_PI;
    for(unsigned int i = 0 ; i < rmsg.laserSensors.size() ; i++)
    {
      rmsg.laserSensors[i].maxAngle = 
        rmsg.laserSensors[i].maxAngle * 180.0 / STDR_PI;
      rmsg.laserSensors[i].minAngle = 
        rmsg.laserSensors[i].minAngle * 180.0 / STDR_PI;
      rmsg.laserSensors[i].pose.theta = 
        rmsg.laserSensors[i].pose.theta * 180.0 / STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.sonarSensors.size() ; i++)
    {
      rmsg.sonarSensors[i].coneAngle = 
        rmsg.sonarSensors[i].coneAngle * 180.0 / STDR_PI;
      rmsg.sonarSensors[i].pose.theta = 
        rmsg.sonarSensors[i].pose.theta * 180.0 / STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.rfidSensors.size() ; i++)
    {
      rmsg.rfidSensors[i].angleSpan = 
        rmsg.rfidSensors[i].angleSpan * 180.0 / STDR_PI;
      rmsg.rfidSensors[i].pose.theta = 
        rmsg.rfidSensors[i].pose.theta * 180.0 / STDR_PI;
    }
    return rmsg;
  }
}
