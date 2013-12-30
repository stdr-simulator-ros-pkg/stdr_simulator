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

#include "stdr_xml_parser/stdr_xml_parser_msg_creator.h"

namespace stdr_xml_parser
{
  
  MessageCreator::MessageCreator(void)
  {

  }
  
  stdr_msgs::RobotMsg MessageCreator::createRobotMessage(Node *n)
  {
    stdr_msgs::RobotMsg msg;
    Node* specs = n->elements[0]->elements[0];
    std::vector<int> indexes;
    
    //!< Search for pose
    indexes = specs->getTag("initial_pose");
    if(indexes.size() != 0)
    {
      msg.initialPose = createPoseMessage(specs->elements[indexes[0]]);
    }
    else
    {
      msg.initialPose.x = atof(Specs::specs["x"].default_value.c_str());
      msg.initialPose.y = atof(Specs::specs["y"].default_value.c_str());
      msg.initialPose.theta = atof(Specs::specs["theta"].default_value.c_str());
    }
    
    //!< Search for footprint
    indexes = specs->getTag("footprint");
    if(indexes.size() != 0)
    {
      msg.footprint = createFootprintMessage(specs->elements[indexes[0]]);
    }
    else
    {
      msg.footprint.radius = atof(Specs::specs["radius"].default_value.c_str());
    }
    
    //!< Search for laser sensors
    indexes = specs->getTag("laser");
    if(indexes.size() != 0)
    {
      for(unsigned int i = 0 ; i < indexes.size() ; i++)
      {
        msg.laserSensors.push_back(createLaserMessage(
          specs->elements[indexes[i]] , i));
      }
    }
    
    //!< Search for sonar sensors
    indexes = specs->getTag("sonar");
    if(indexes.size() != 0)
    {
      for(unsigned int i = 0 ; i < indexes.size() ; i++)
      {
        msg.sonarSensors.push_back(createSonarMessage(
          specs->elements[indexes[i]] , i));
      }
    }
    
    return msg;
  }
  
  stdr_msgs::LaserSensorMsg MessageCreator::createLaserMessage(
    Node *n,unsigned int id)
  {
    stdr_msgs::LaserSensorMsg msg;
    Node* specs = n->elements[0];
    std::vector<int> indexes;
    
    //!< Search for max angle
    indexes = specs->getTag("max_angle");
    if(indexes.size() == 0)
    {
      msg.maxAngle = atof(Specs::specs["max_angle"].default_value.c_str());
    }
    else
    {
      msg.maxAngle = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for min angle
    indexes = specs->getTag("min_angle");
    if(indexes.size() == 0)
    {
      msg.minAngle = atof(Specs::specs["min_angle"].default_value.c_str());
    }
    else
    {
      msg.minAngle = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for max range
    indexes = specs->getTag("max_range");
    if(indexes.size() == 0)
    {
      msg.maxRange = atof(Specs::specs["max_range"].default_value.c_str());
    }
    else
    {
      msg.maxRange = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for max range
    indexes = specs->getTag("min_range");
    if(indexes.size() == 0)
    {
      msg.minRange = atof(Specs::specs["min_range"].default_value.c_str());
    }
    else
    {
      msg.minRange = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for numRays
    indexes = specs->getTag("num_rays");
    if(indexes.size() == 0)
    {
      msg.numRays = atoi(Specs::specs["num_rays"].default_value.c_str());
    }
    else
    {
      msg.numRays = atoi(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for noise
    indexes = specs->getTag("noise");
    if(indexes.size() != 0)
    {
      msg.noise = createNoiseMessage(specs->elements[indexes[0]]);
    }
    
    //!< Search for frequency
    indexes = specs->getTag("frequency");
    if(indexes.size() == 0)
    {
      msg.frequency = atof(Specs::specs["frequency"].default_value.c_str());
    }
    else
    {
      msg.frequency = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Set up frame id based on id input
    msg.frame_id = std::string("laser_") + SSTR(id);
    
    //!< Search for pose
    indexes = specs->getTag("pose");
    if(indexes.size() != 0)
    {
      msg.pose = createPoseMessage(specs->elements[indexes[0]]);
    }
    else
    {
      msg.pose.x = atof(Specs::specs["x"].default_value.c_str());
      msg.pose.y = atof(Specs::specs["y"].default_value.c_str());
      msg.pose.theta = atof(Specs::specs["theta"].default_value.c_str());
    }
    return msg;
  }
  
  stdr_msgs::SonarSensorMsg MessageCreator::createSonarMessage(
    Node *n,unsigned int id)
  {
    stdr_msgs::SonarSensorMsg msg;
    Node* specs = n->elements[0];
    std::vector<int> indexes;
    
    //!< Search for max range
    indexes = specs->getTag("max_range");
    if(indexes.size() == 0)
    {
      msg.maxRange = atof(Specs::specs["max_range"].default_value.c_str());
    }
    else
    {
      msg.maxRange = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for max range
    indexes = specs->getTag("min_range");
    if(indexes.size() == 0)
    {
      msg.minRange = atof(Specs::specs["min_range"].default_value.c_str());
    }
    else
    {
      msg.minRange = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for coneAngle
    indexes = specs->getTag("cone_angle");
    if(indexes.size() == 0)
    {
      msg.coneAngle = atof(Specs::specs["cone_angle"].default_value.c_str());
    }
    else
    {
      msg.coneAngle = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Search for noise
    indexes = specs->getTag("noise");
    if(indexes.size() != 0)
    {
      msg.noise = createNoiseMessage(specs->elements[indexes[0]]);
    }
    
    //!< Search for frequency
    indexes = specs->getTag("frequency");
    if(indexes.size() == 0)
    {
      msg.frequency = atof(Specs::specs["frequency"].default_value.c_str());
    }
    else
    {
      msg.frequency = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    
    //!< Set up frame id based on id input
    msg.frame_id = std::string("sonar_") + SSTR(id);
    
    //!< Search for pose
    indexes = specs->getTag("pose");
    if(indexes.size() != 0)
    {
      msg.pose = createPoseMessage(specs->elements[indexes[0]]);
    }
    else
    {
      msg.pose.x = atof(Specs::specs["x"].default_value.c_str());
      msg.pose.y = atof(Specs::specs["y"].default_value.c_str());
      msg.pose.theta = atof(Specs::specs["theta"].default_value.c_str());
    }
    return msg;
  }
  
  stdr_msgs::FootprintMsg MessageCreator::createFootprintMessage(Node *n)
  {
    stdr_msgs::FootprintMsg msg;
    Node* specs = n->elements[0];
    std::vector<int> indexes;
    //!< Search for radius
    indexes = specs->getTag("radius");
    if(indexes.size() == 0)
    {
      msg.radius = atof(Specs::specs["radius"].default_value.c_str());
    }
    else
    {
      msg.radius = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    return msg;
  }
  
  stdr_msgs::Noise MessageCreator::createNoiseMessage(Node *n)
  {
    stdr_msgs::Noise msg;
    Node* specs = n->elements[0];
    std::vector<int> indexes;
    //!< Search for noise mean
    indexes = specs->getTag("noise_mean");
    if(indexes.size() == 0)
    {
      msg.noiseMean = atof(Specs::specs["noise_mean"].default_value.c_str());
    }
    else
    {
      msg.noiseMean = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    //!< Search for noise std
    indexes = specs->getTag("noise_std");
    if(indexes.size() == 0)
    {
      msg.noiseStd = atof(Specs::specs["noise_std"].default_value.c_str());
    }
    else
    {
      msg.noiseStd = atof(specs->elements[indexes[0]]->elements[0]->
        value.c_str());
    }
    return msg;
  }
  
  geometry_msgs::Pose2D MessageCreator::createPoseMessage(Node *n)
  {
    geometry_msgs::Pose2D msg;
    std::vector<int> indexes;
    //!< Search for x
    indexes = n->getTag("x");
    if(indexes.size() == 0)
    {
      msg.x = atof(Specs::specs["x"].default_value.c_str());
    }
    else
    {
      msg.x = atof(n->elements[indexes[0]]->elements[0]->value.c_str());
    }
    //!< Search for y
    indexes = n->getTag("y");
    if(indexes.size() == 0)
    {
      msg.y = atof(Specs::specs["y"].default_value.c_str());
    }
    else
    {
      msg.y = atof(n->elements[indexes[0]]->elements[0]->value.c_str());
    }
    //!< Search for theta
    indexes = n->getTag("theta");
    if(indexes.size() == 0)
    {
      msg.theta = atof(Specs::specs["theta"].default_value.c_str());
    }
    else
    {
      msg.theta = atof(n->elements[indexes[0]]->elements[0]->value.c_str());
    }
    return msg;
  }
}

