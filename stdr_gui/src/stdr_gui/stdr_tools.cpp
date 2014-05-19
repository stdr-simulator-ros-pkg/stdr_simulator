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
  @param rmsg [stdr_msgs::RobotMsg] The robot message
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
    for(unsigned int i = 0 ; i < rmsg.co2Sensors.size() ; i++)
    {
      //~ rmsg.co2Sensors[i].angleSpan = 
        //~ rmsg.co2Sensors[i].angleSpan / 180.0 * STDR_PI;
      rmsg.co2Sensors[i].pose.theta = 
        rmsg.co2Sensors[i].pose.theta / 180.0 * STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.thermalSensors.size() ; i++)
    {
      rmsg.thermalSensors[i].angleSpan = 
        rmsg.thermalSensors[i].angleSpan / 180.0 * STDR_PI;
      rmsg.thermalSensors[i].pose.theta = 
        rmsg.thermalSensors[i].pose.theta / 180.0 * STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.soundSensors.size() ; i++)
    {
      rmsg.soundSensors[i].angleSpan = 
        rmsg.soundSensors[i].angleSpan / 180.0 * STDR_PI;
      rmsg.soundSensors[i].pose.theta = 
        rmsg.soundSensors[i].pose.theta / 180.0 * STDR_PI;
    }
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::RobotMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::RobotMsg] The robot message
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
    for(unsigned int i = 0 ; i < rmsg.co2Sensors.size() ; i++)
    {
      //~ rmsg.rfidSensors[i].angleSpan = 
        //~ rmsg.rfidSensors[i].angleSpan * 180.0 / STDR_PI;
      rmsg.co2Sensors[i].pose.theta = 
        rmsg.co2Sensors[i].pose.theta * 180.0 / STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.thermalSensors.size() ; i++)
    {
      rmsg.thermalSensors[i].angleSpan = 
        rmsg.thermalSensors[i].angleSpan * 180.0 / STDR_PI;
      rmsg.thermalSensors[i].pose.theta = 
        rmsg.thermalSensors[i].pose.theta * 180.0 / STDR_PI;
    }
    for(unsigned int i = 0 ; i < rmsg.soundSensors.size() ; i++)
    {
      rmsg.soundSensors[i].angleSpan = 
        rmsg.soundSensors[i].angleSpan * 180.0 / STDR_PI;
      rmsg.soundSensors[i].pose.theta = 
        rmsg.soundSensors[i].pose.theta * 180.0 / STDR_PI;
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
  @brief Takes a stdr_msgs::RfidSensorMsg and converts its angles to rads
  @param rmsg [stdr_msgs::RfidSensorMsg] The rfid reader message
  @return stdr_msgs::RfidSensorMsg : The recreated rfid reader message
  **/
  stdr_msgs::RfidSensorMsg fixRfidAnglesToRad(stdr_msgs::RfidSensorMsg rmsg)
  {
    rmsg.angleSpan = rmsg.angleSpan / 180.0 * STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta / 180.0 * STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::RfidSensorMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::RfidSensorMsg] The rfid reader message
  @return stdr_msgs::RfidSensorMsg : The recreated rfid reader message
  **/
  stdr_msgs::RfidSensorMsg fixRfidAnglesToDegrees(
    stdr_msgs::RfidSensorMsg rmsg)
  {
    rmsg.angleSpan = rmsg.angleSpan * 180.0 / STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta * 180.0 / STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::CO2SensorMsg and converts its angles to rads
  **/
  stdr_msgs::CO2SensorMsg fixCO2AnglesToRad(stdr_msgs::CO2SensorMsg rmsg)
  {
    //~ rmsg.angleSpan = rmsg.angleSpan / 180.0 * STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta / 180.0 * STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::CO2SensorMsg and converts its angles to degrees
  **/
  stdr_msgs::CO2SensorMsg fixCO2AnglesToDegrees(
    stdr_msgs::CO2SensorMsg rmsg)
  {
    //~ rmsg.angleSpan = rmsg.angleSpan * 180.0 / STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta * 180.0 / STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::ThermalSensorMsg and converts its angles to rads
  **/
  stdr_msgs::ThermalSensorMsg fixThermalAnglesToRad(stdr_msgs::ThermalSensorMsg rmsg)
  {
    rmsg.angleSpan = rmsg.angleSpan / 180.0 * STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta / 180.0 * STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::ThermalSensorMsg and converts its angles to degrees
  **/
  stdr_msgs::ThermalSensorMsg fixThermalAnglesToDegrees(
    stdr_msgs::ThermalSensorMsg rmsg)
  {
    rmsg.angleSpan = rmsg.angleSpan * 180.0 / STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta * 180.0 / STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::SoundSensorMsg and converts its angles to rads
  **/
  stdr_msgs::SoundSensorMsg fixSoundAnglesToRad(stdr_msgs::SoundSensorMsg rmsg)
  {
    rmsg.angleSpan = rmsg.angleSpan / 180.0 * STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta / 180.0 * STDR_PI;
    return rmsg;
  }
  
  /**
  @brief Takes a stdr_msgs::SoundSensorMsg and converts its angles to degrees
  **/
  stdr_msgs::SoundSensorMsg fixSoundAnglesToDegrees(
    stdr_msgs::SoundSensorMsg rmsg)
  {
    rmsg.angleSpan = rmsg.angleSpan * 180.0 / STDR_PI;
    rmsg.pose.theta = rmsg.pose.theta * 180.0 / STDR_PI;
    return rmsg;
  }
}
