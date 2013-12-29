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
  /**
  @brief Returns the global path of the ROS package provided
  @param package [std::string] The ROS package
  @return std::string : The global path of the specific package
  **/
  std::string getRosPackagePath(std::string package)
  {
    return ros::package::getPath(package.c_str());
  }
  
  /**
  @brief Converts an angle from rads to degrees
  @param angle [float] An angle in rads
  @return float : The angle in degrees
  **/
  float angleRadToDegrees(float angle)
  {
    return angle * 180.0 / STDR_PI;
  }
  
  /**
  @brief Converts an angle from degrees to rads
  @param angle [float] An angle in degrees
  @return float : The angle in rads
  **/
  float angleDegreesToRad(float angle)
  {
    return angle / 180.0 * STDR_PI;
  }
  
  /**
  @brief Transforms the milliseconds in literal representation
  @param ms [int] The time in ms
  @return QString : The literal representation of time given
  **/
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
  
  /**
  @brief Prints a sonar msg
  @param msg [stdr_msgs::SonarSensorMsg &] The message
  @return void
  **/
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
  
  /**
  @brief Prints a laser msg
  @param msg [stdr_msgs::LaserSensorMsg &] The message
  @return void
  **/
  void printLaserMsg(stdr_msgs::LaserSensorMsg &msg)
  {
    ROS_ERROR("Laser sensor msg :");
    ROS_ERROR("\tMax range : %f",msg.maxRange);
    ROS_ERROR("\tMin range : %f",msg.minRange);
    ROS_ERROR("\tMax angle : %f",msg.maxAngle);
    ROS_ERROR("\tMin angle : %f",msg.minAngle);
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
  
  /**
  @brief Prints a ROS pose2d msg
  @param msg [geometry_msgs::Pose2D &] The message
  @return void
  **/
  void printPose2D(geometry_msgs::Pose2D &msg)
  {
    ROS_ERROR("Pose 2D :");
    ROS_ERROR("\tx : %f",msg.x);
    ROS_ERROR("\ty : %f",msg.y);
    ROS_ERROR("\ttheta : %f",msg.theta);
  }
  
  /**
  @brief Takes a stdr_msgs::RobotMsg and converts its angles to rads
  @param robot [stdr_msgs::RobotMsg] The robot message
  @return stdr_msgs::RobotMsg : The recreated robot message
  **/
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
  
  /**
  @brief Takes a stdr_msgs::RobotMsg and converts its angles to degrees
  @param robot [stdr_msgs::RobotMsg] The robot message
  @return stdr_msgs::RobotMsg : The recreated robot message
  **/
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
  
  /**
  @brief Takes a stdr_msgs::LaserSensorMsg and converts its angles to rads
  @param rmsg [stdr_msgs::LaserSensorMsg] The laser message
  @return stdr_msgs::LaserSensorMsg : The recreated laser message
  **/
  stdr_msgs::LaserSensorMsg fixLaserAnglesToRad(stdr_msgs::LaserSensorMsg rmsg)
  {
    rmsg.maxAngle = rmsg.maxAngle / 180.0 * STDR_PI;
    rmsg.minAngle = rmsg.minAngle / 180.0 * STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta / 180.0 * STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::LaserSensorMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::LaserSensorMsg] The laser message
  @return stdr_msgs::LaserSensorMsg : The recreated laser message
  **/
  stdr_msgs::LaserSensorMsg fixLaserAnglesToDegrees(stdr_msgs::LaserSensorMsg rmsg)
  {
    rmsg.maxAngle = rmsg.maxAngle * 180.0 / STDR_PI;
    rmsg.minAngle = rmsg.minAngle * 180.0 / STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta * 180.0 / STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::SonarSensorMsg and converts its angles to rads
  @param rmsg [stdr_msgs::SonarSensorMsg] The sonar message
  @return stdr_msgs::SonarSensorMsg : The recreated sonar message
  **/
  stdr_msgs::SonarSensorMsg fixSonarAnglesToRad(stdr_msgs::SonarSensorMsg rmsg)
  {
    rmsg.coneAngle = rmsg.coneAngle / 180.0 * STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta / 180.0 * STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::SonarSensorMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::SonarSensorMsg] The sonar message
  @return stdr_msgs::SonarSensorMsg : The recreated sonar message
  **/
  stdr_msgs::SonarSensorMsg fixSonarAnglesToDegrees(
    stdr_msgs::SonarSensorMsg rmsg)
  {
    rmsg.coneAngle = rmsg.coneAngle * 180.0 / STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta * 180.0 / STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Transforms a quaternion angle message to euler angles
  @param msg [geometry_msgs::Quaternion] A ROS quaternion message
  @return std::vector<float> : The euler angles
  **/
  std::vector<float> quaternionToEuler(geometry_msgs::Quaternion msg)
  {
    float qx = msg.x;
    float qy = msg.y;
    float qz = msg.z;
    float qw = msg.w;
    float rx = atan2(2*((qw * qx) + (qy * qz)), 1 - (2 * ((qx* qx) + (qy * qy))));
    float ry = asin(2 * ((qw * qy) - (qz * qx)));
    float rz = atan2(2 * ((qw * qz) + (qx * qy)), 1 - (2 * ((qy * qy) + (qz * qz))));
    std::vector<float> eu;
    eu.push_back(rx);
    eu.push_back(ry);
    eu.push_back(rz);
    return eu;
  }
  
  /**
  @brief Transforms a point based on origin from local map to global coordinate system
  @param origin [geometry_msgs::Pose] The map origin
  @param p [geometry_msgs::Pose2D] The point to be transformed
  @return geometry_msgs::Pose2D : The transformed point
  **/
  geometry_msgs::Pose2D guiToGlobal(
    geometry_msgs::Pose origin,geometry_msgs::Pose2D p)
  {
    float x = p.x;
    float y = p.y;
    float orx = origin.position.x;
    float ory = origin.position.y;
    std::vector<float> eu = quaternionToEuler(origin.orientation);
    float orth = eu[2];
    geometry_msgs::Pose2D ret;
    ret.x = (cos(orth) * x - sin(orth) * y) + orx;
    ret.y = (sin(orth) * x + cos(orth) * y) + ory;
    ret.theta = p.theta + eu[2];
    return ret;
  }
  
  /**
  @brief Transforms a point based on origin from global coordinate system to the map
  @param origin [geometry_msgs::Pose] The map origin
  @param p [geometry_msgs::Pose2D] The point to be transformed
  @return geometry_msgs::Pose2D : The transformed point
  **/
  geometry_msgs::Pose2D globalToGui(
    geometry_msgs::Pose origin,geometry_msgs::Pose2D p)
  {
    float x = p.x;
    float y = p.y;
    float orx = origin.position.x;
    float ory = origin.position.y;
    std::vector<float> eu = quaternionToEuler(origin.orientation);
    float orth = eu[2];
    geometry_msgs::Pose2D ret;
    ret.x = (cos(orth) * (x - orx) + sin(orth) * (y - ory));
    ret.y = (- sin(orth) * (x - orx) + cos(orth) * (y - ory));
    ret.theta = p.theta - eu[2];
    return ret;
  }
}
