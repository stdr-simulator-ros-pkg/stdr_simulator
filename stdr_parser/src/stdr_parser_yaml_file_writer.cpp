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

#include "stdr_parser/stdr_parser_yaml_file_writer.h"

namespace stdr_parser
{
  
  /**
  @brief Default constructor
  @return void
  **/
  YamlFileWriter::YamlFileWriter(void)
  {

  }
  
  //!<----------------------------------------------------------------
  //!< Template declaration for stdr_msgs::Noise
  template void YamlFileWriter::messageToFile
    (stdr_msgs::Noise msg,std::string file_name);
  
  //!< Template specialization for stdr_msgs::Noise
  template <>
  YAML::Emitter& operator << <stdr_msgs::Noise>
    (YAML::Emitter& out, const stdr_msgs::Noise& msg)
  {
    out << YAML::BeginMap;
      out << YAML::Key << "noise";
      out << YAML::Value;
      out << YAML::BeginMap;
        out << YAML::Key << "noise_specifications";
        out << YAML::Value;
        out << YAML::BeginMap;
          out << YAML::Key << "noise_mean" << YAML::Value << msg.noiseMean;
          out << YAML::Key << "noise_std" << YAML::Value << msg.noiseStd;
        out << YAML::EndMap;
      out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }
  
  //!<----------------------------------------------------------------
  //!< Template declaration for stdr_msgs::FootprintMsg
  template void YamlFileWriter::messageToFile
    (stdr_msgs::FootprintMsg msg,std::string file_name);
  
  //!< Template specialization for stdr_msgs::FootprintMsg
  template <>
  YAML::Emitter& operator << <stdr_msgs::FootprintMsg>
    (YAML::Emitter& out, const stdr_msgs::FootprintMsg& msg)
  {
    out << YAML::BeginMap;
      out << YAML::Key << "footprint";
      out << YAML::Value;
      out << YAML::BeginMap;
        out << YAML::Key << "footprint_specifications";
        out << YAML::Value;
        out << YAML::BeginMap;
          out << YAML::Key << "radius" << YAML::Value << msg.radius;
        out << YAML::EndMap;
      out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }
  
  //!<----------------------------------------------------------------
  //!< Template declaration for geometry_msgs::Pose2D
  template void YamlFileWriter::messageToFile
    (geometry_msgs::Pose2D msg,std::string file_name);
  
  //!< Template specialization for geometry_msgs::Pose2D
  template <>
  YAML::Emitter& operator << <geometry_msgs::Pose2D>
    (YAML::Emitter& out, const geometry_msgs::Pose2D& msg)
  {
    out << YAML::BeginMap;
      out << YAML::Key << "pose";
      out << YAML::Value;
      out << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << msg.x;
        out << YAML::Key << "y" << YAML::Value << msg.y;
        out << YAML::Key << "theta" << YAML::Value << msg.theta;
      out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }
  
  //!<----------------------------------------------------------------
  //!< Template declaration for stdr_msgs::LaserSensorMsg
  template void YamlFileWriter::messageToFile
    (stdr_msgs::LaserSensorMsg msg,std::string file_name);
  
  //!< Template specialization for stdr_msgs::LaserSensorMsg
  template <>
  YAML::Emitter& operator << <stdr_msgs::LaserSensorMsg>
    (YAML::Emitter& out, const stdr_msgs::LaserSensorMsg& msg)
  {
    out << YAML::BeginMap;
      out << YAML::Key << "laser";
      out << YAML::Value;
      out << YAML::BeginMap;
        out << YAML::Key << "laser_specifications";
        out << YAML::Value;
        out << YAML::BeginMap;
          out << YAML::Key << "max_angle" << YAML::Value << msg.maxAngle;
          out << YAML::Key << "min_angle" << YAML::Value << msg.minAngle;
          out << YAML::Key << "max_range" << YAML::Value << msg.maxRange;
          out << YAML::Key << "min_range" << YAML::Value << msg.minRange;
          out << YAML::Key << "num_rays" << YAML::Value << msg.numRays;
          out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
          out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
          out << YAML::Key << "pose";
          out << YAML::Value;
          out << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << msg.pose.x;
            out << YAML::Key << "y" << YAML::Value << msg.pose.y;
            out << YAML::Key << "theta" << YAML::Value << msg.pose.theta;
          out << YAML::EndMap;
          out << YAML::Key << "noise";
          out << YAML::Value;
          out << YAML::BeginMap;
            out << YAML::Key << "noise_specifications";
            out << YAML::Value;
            out << YAML::BeginMap;
              out << YAML::Key << "noise_mean";
              out << YAML::Value << msg.noise.noiseMean;
              out << YAML::Key << "noise_std";
              out << YAML::Value << msg.noise.noiseStd;
            out << YAML::EndMap;
          out << YAML::EndMap;
        out << YAML::EndMap;
      out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }
  
  //!<----------------------------------------------------------------
  //!< Template declaration for stdr_msgs::SonarSensorMsg
  template void YamlFileWriter::messageToFile
    (stdr_msgs::SonarSensorMsg msg,std::string file_name);
  
  //!< Template specialization for stdr_msgs::SonarSensorMsg
  template <>
  YAML::Emitter& operator << <stdr_msgs::SonarSensorMsg>
    (YAML::Emitter& out, const stdr_msgs::SonarSensorMsg& msg)
  {
    out << YAML::BeginMap;
      out << YAML::Key << "sonar";
      out << YAML::Value;
      out << YAML::BeginMap;
        out << YAML::Key << "sonar_specifications";
        out << YAML::Value;
        out << YAML::BeginMap;
          out << YAML::Key << "cone_angle" << YAML::Value << msg.coneAngle;
          out << YAML::Key << "max_range" << YAML::Value << msg.maxRange;
          out << YAML::Key << "min_range" << YAML::Value << msg.minRange;
          out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
          out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
          out << YAML::Key << "pose";
          out << YAML::Value;
          out << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << msg.pose.x;
            out << YAML::Key << "y" << YAML::Value << msg.pose.y;
            out << YAML::Key << "theta" << YAML::Value << msg.pose.theta;
          out << YAML::EndMap;
          out << YAML::Key << "noise";
          out << YAML::Value;
          out << YAML::BeginMap;
            out << YAML::Key << "noise_specifications";
            out << YAML::Value;
            out << YAML::BeginMap;
              out << YAML::Key << "noise_mean";
              out << YAML::Value << msg.noise.noiseMean;
              out << YAML::Key << "noise_std";
              out << YAML::Value << msg.noise.noiseStd;
            out << YAML::EndMap;
          out << YAML::EndMap;
        out << YAML::EndMap;
      out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }
  
  //!<----------------------------------------------------------------
  //!< Template declaration for stdr_msgs::RobotMsg
  template void YamlFileWriter::messageToFile
    (stdr_msgs::RobotMsg msg,std::string file_name);
  
  //!< Template specialization for stdr_msgs::RobotMsg
  template <>
  YAML::Emitter& operator << <stdr_msgs::RobotMsg>
    (YAML::Emitter& out, const stdr_msgs::RobotMsg& msg)
  {
    out << YAML::BeginMap;
      out << YAML::Key << "robot";
      out << YAML::Value;
      out << YAML::BeginMap;
        out << YAML::Key << "robot_specifications";
        out << YAML::Value;
        out << YAML::BeginSeq;
          out << msg.footprint;
          out << YAML::BeginMap;
            out << YAML::Key << "initial_pose";
            out << YAML::Value;
            out << YAML::BeginMap;
              out << YAML::Key << "x" << YAML::Value << msg.initialPose.x;
              out << YAML::Key << "y" << YAML::Value << msg.initialPose.y;
              out << YAML::Key << "theta" << YAML::Value << msg.initialPose.theta;
            out << YAML::EndMap;
          out << YAML::EndMap;
          for(unsigned int i = 0 ; i < msg.laserSensors.size() ; i++)
          {
            out << msg.laserSensors[i];
          }
          for(unsigned int i = 0 ; i < msg.sonarSensors.size() ; i++)
          {
            out << msg.sonarSensors[i];
          }
        out << YAML::EndSeq;
      out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }
  //!<----------------------------------------------------------------
  
  /**
  @brief Creates an yaml file from a message - template member function
  @param msg [T] The message
  @param file_name [std::string] The yaml file name to write the message
  @return void
  **/
  template <class T>
  void YamlFileWriter::messageToFile(T msg,std::string file_name)
  {
    YAML::Emitter out;
    out << msg;
    
    std::ofstream sensorYamlFile;    
    sensorYamlFile.open(file_name.c_str());
    sensorYamlFile << out.c_str();
    sensorYamlFile.close();
  }

}

