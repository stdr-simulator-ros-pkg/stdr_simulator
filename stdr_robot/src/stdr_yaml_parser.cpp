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

#include "stdr_robot/stdr_yaml_parser.h"
#include "ros/ros.h"

namespace stdr_robot {
  
  namespace parser {
  
  stdr_msgs::RobotMsg yamlToRobotMsg(const std::string& filename) {
    
    std::ifstream fin(filename.c_str());
    
    #ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
    
    stdr_msgs::RobotMsg robot;
    
    //~ doc["robots"][0] >> robot; //multiple robots
    doc >> robot; 
    
    return robot;
    
  }
  
  stdr_msgs::LaserSensorMsg yamlToLaserSensorMsg(const std::string& filename) {
    std::ifstream fin(filename.c_str());
    
    #ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
    
    stdr_msgs::LaserSensorMsg laser;
    
    doc >> laser;
    
    return laser;
  }
  
  stdr_msgs::SonarSensorMsg yamlToSonarSensorMsg(const std::string& filename) {
    std::ifstream fin(filename.c_str());
    
    #ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
    
    stdr_msgs::SonarSensorMsg sonar;
    
    doc >> sonar;
    
    return sonar;
  }
  
  stdr_msgs::RfidSensorMsg yamlToRfidSensorMsg(const std::string& filename) {
    std::ifstream fin(filename.c_str());
    
    #ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
    #else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
    #endif
    
    stdr_msgs::RfidSensorMsg rfid;
    
    doc >> rfid;
    
    return rfid;
  }

  void robotMsgToYaml(const std::string& filename, const stdr_msgs::RobotMsg& msg) {
    
    YAML::Emitter out;
    
    out << msg;
    
    std::ofstream robotYamlFile;
    
    robotYamlFile.open(filename.c_str());
    
    robotYamlFile << out.c_str();
    
    robotYamlFile.close();
  }
  
  void laserSensorMsgToYaml(const std::string& filename, const stdr_msgs::LaserSensorMsg& msg) {
    
    YAML::Emitter out;
    
    out << msg;
    
    std::ofstream sensorYamlFile;
    
    sensorYamlFile.open(filename.c_str());
    
    sensorYamlFile << out.c_str();
    
    sensorYamlFile.close();
  }
  
  void sonarSensorMsgToYaml(const std::string& filename, const stdr_msgs::SonarSensorMsg& msg) {
    
    YAML::Emitter out;
    
    out << msg;
    
    std::ofstream sensorYamlFile;
    
    sensorYamlFile.open(filename.c_str());
    
    sensorYamlFile << out.c_str();
    
    sensorYamlFile.close();
  }
  
  void rfidSensorMsgToYaml(const std::string& filename, const stdr_msgs::RfidSensorMsg& msg) {
    
    
    YAML::Emitter out;
    
    out << msg;
    
    std::ofstream sensorYamlFile;
    
    sensorYamlFile.open(filename.c_str());
    
    sensorYamlFile << out.c_str();
    
    sensorYamlFile.close();
  }

  // operators for parsing
  void operator >> (const YAML::Node& node, stdr_msgs::RobotMsg& msg) {
    node["initial_pose"] >> msg.initialPose;
    node["footprint"] >> msg.footprint;
    for (YAML::Iterator it = node["sensors"].begin(); it != node["sensors"].end(); ++it) {
      std::string sensorType;
      it.first() >> sensorType;
      for (unsigned i = 0; i < it.second().size(); i++) {
        if (sensorType == "lasers") {
          stdr_msgs::LaserSensorMsg laser;
          it.second()[i] >> laser;
          msg.laserSensors.push_back(laser);
        }
        
        if (sensorType == "sonars") {
          stdr_msgs::SonarSensorMsg sonar;
          it.second()[i] >> sonar;
          msg.sonarSensors.push_back(sonar);
        }
        
        if (sensorType == "rfids") {
          stdr_msgs::RfidSensorMsg rfid;
          it.second()[i] >> rfid;
          msg.rfidSensors.push_back(rfid);
        }
      }
    }
    node["kinematic"] >> msg.kinematicModel;
  }
  
  void operator >> (const YAML::Node& node, geometry_msgs::Pose2D& msg) {
    node["x"] >> msg.x;
    node["y"] >> msg.y;
    node["theta"] >> msg.theta;
  }
  
  void operator >> (const YAML::Node& node, stdr_msgs::LaserSensorMsg& msg) {
    node["max_angle"] >> msg.maxAngle;
    node["min_angle"] >> msg.minAngle;
    node["max_range"] >> msg.maxRange;
    if (msg.maxRange <= 0)
      throw YAML::RepresentationException(node["max_range"].GetMark(), "negative or zero value");
    node["min_range"] >> msg.minRange;
    if (msg.minRange < 0)
      throw YAML::RepresentationException(node["min_range"].GetMark(), "negative or zero value");
    node["num_rays"] >> msg.numRays;
    if (msg.numRays <= 0)
      throw YAML::RepresentationException(node["num_rays"].GetMark(), "negative or zero value");
    
    if(const YAML::Node *noise = node.FindValue("noise")) {
      *noise >> msg.noise;
    }
    
    node["frequency"] >> msg.frequency;
    if (msg.frequency <= 0)
      throw YAML::RepresentationException(node["frequency"].GetMark(), "negative or zero value");
    node["frame_id"] >> msg.frame_id;
    node["pose"] >> msg.pose;
  }
  
  void operator >> (const YAML::Node& node, stdr_msgs::SonarSensorMsg& msg) {
    node["cone_angle"] >> msg.coneAngle;
    node["max_range"] >> msg.maxRange;
    if (msg.maxRange <= 0)
      throw YAML::RepresentationException(node["max_range"].GetMark(), "negative or zero value");
    node["min_range"] >> msg.minRange;
    if (msg.minRange < 0)
      throw YAML::RepresentationException(node["min_range"].GetMark(), "negative or zero value");
    node["frequency"] >> msg.frequency;
    if (msg.frequency <= 0)
      throw YAML::RepresentationException(node["frequency"].GetMark(), "negative or zero value");
    if(const YAML::Node *noise = node.FindValue("noise")) {
      *noise >> msg.noise;
    }
    node["frame_id"] >> msg.frame_id;
    node["pose"] >> msg.pose;
  }
  
  void operator >> (const YAML::Node& node, stdr_msgs::RfidSensorMsg& msg) {
    node["max_range"] >> msg.maxRange;
    node["angle_span"] >> msg.angleSpan;
    node["signal_cutoff"] >> msg.signalCutoff;
    node["frequency"] >> msg.frequency;
    if (msg.frequency <= 0)
      throw YAML::RepresentationException(node["frequency"].GetMark(), "negative or zero value");
    node["frame_id"] >> msg.frame_id;
    node["pose"] >> msg.pose;
  }
  
  void operator >> (const YAML::Node& node, stdr_msgs::FootprintMsg& msg) {
    if(const YAML::Node *rad = node.FindValue("radius")) {
      *rad >> msg.radius;
    }
    else if (const YAML::Node *points = node.FindValue("points")) {
      for (int i = 0; i < points->size(); i++) {
        geometry_msgs::Point pnt;
        (*points)[i] >> pnt;
        msg.points.push_back(pnt);
      }
    }
  }
  
  void operator >> (const YAML::Node& node, geometry_msgs::Point& msg) {
    node[0] >> msg.x;
    node[1] >> msg.y;
  }
  
  void operator >> (const YAML::Node& node, stdr_msgs::Noise& msg) {
    node["mean"] >> msg.noiseMean;
    node["std"] >> msg.noiseStd;
  }
  
  void operator >> (const YAML::Node& node, stdr_msgs::KinematicMsg& msg) {
    node["type"] >> msg.type;
  }

  // operators for emitting
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::RobotMsg& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "initial_pose" << YAML::Value << msg.initialPose;
    out << YAML::Key << "footprint" << YAML::Value << msg.footprint;
    out << YAML::Key << "kinematic" << YAML::Value << msg.kinematicModel;
    out << YAML::Key << "sensors";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "lasers" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < msg.laserSensors.size(); i++) {
      out << msg.laserSensors[i];
    }
    out << YAML::EndSeq;
    out << YAML::Key << "sonars" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < msg.sonarSensors.size(); i++) {
      out << msg.sonarSensors[i];
    }
    out << YAML::EndSeq;
    out << YAML::Key << "rfids" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < msg.rfidSensors.size(); i++) {
      out << msg.rfidSensors[i];
    }
    out << YAML::EndSeq << YAML::EndMap;
    out << YAML::EndMap;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Pose2D& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << msg.x;
    out << YAML::Key << "y" << YAML::Value << msg.y;
    out << YAML::Key << "theta" << YAML::Value << msg.theta;
    out << YAML::EndMap;
    return out;
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::LaserSensorMsg& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "max_angle" << YAML::Value << msg.maxAngle;
    out << YAML::Key << "min_angle" << YAML::Value << msg.minAngle;
    out << YAML::Key << "max_range" << YAML::Value << msg.maxRange;
    out << YAML::Key << "min_range" << YAML::Value << msg.minRange;
    out << YAML::Key << "num_rays" << YAML::Value << msg.numRays;
    out << YAML::Key << "noise" << YAML::Value << msg.noise;
    out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
    out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
    out << YAML::Key << "pose" << YAML::Value << msg.pose;
    out << YAML::EndMap;
    return out;
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::SonarSensorMsg& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "max_range" << YAML::Value << msg.maxRange;
    out << YAML::Key << "min_range" << YAML::Value << msg.minRange;
    out << YAML::Key << "cone_angle" << YAML::Value << msg.coneAngle;
    out << YAML::Key << "noise" << YAML::Value << msg.noise;
    out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
    out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
    out << YAML::Key << "pose" << YAML::Value << msg.pose;
    out << YAML::EndMap;
    return out;
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::RfidSensorMsg& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "max_range" << YAML::Value << msg.maxRange;
    out << YAML::Key << "angle_span" << YAML::Value << msg.angleSpan;
    out << YAML::Key << "signal_cutoff" << YAML::Value << msg.signalCutoff;
    out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
    out << YAML::Key << "pose" << YAML::Value << msg.pose;
    out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
    out << YAML::EndMap;
    return out;
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::FootprintMsg& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "radius" << YAML::Value << msg.radius;
    out << YAML::Key << "points";
    out << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < msg.points.size(); i++) {
      out << YAML::Flow << msg.points[i];      
    }
    out << YAML::EndSeq << YAML::EndMap;
    return out;    
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Point& msg) {
    out << YAML::BeginSeq << YAML::Flow << msg.x << msg.y << YAML::EndSeq;
    return out;
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::Noise& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "mean" << YAML::Value << msg.noiseMean;
    out << YAML::Key << "std" << YAML::Value << msg.noiseStd;
    out << YAML::EndMap;
    return out;
  }
  
  YAML::Emitter& operator << (YAML::Emitter& out, const stdr_msgs::KinematicMsg& msg) {
    out << YAML::BeginMap;
    out << YAML::Key << "type" << YAML::Value << msg.type;
    out << YAML::EndMap;
    return out;
  }

  } // end of namespace parser

} // end of namespace stdr_robot
