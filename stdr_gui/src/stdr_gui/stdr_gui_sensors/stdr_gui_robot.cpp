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

#include "stdr_gui/stdr_gui_sensors/stdr_gui_robot.h"

namespace stdr_gui
{
  /**
  @brief Default contructor
  @param msg [const stdr_msgs::RobotIndexedMsg&] The robot description msg
  @return void
  **/
  CGuiRobot::CGuiRobot(const stdr_msgs::RobotIndexedMsg& msg)
  {
    robot_initialized_ = false;
    started_ = false;
    initial_pose_ = msg.robot.initialPose;
    current_pose_ = initial_pose_;
    footprint_ = msg.robot.footprint;
    radius_ = msg.robot.footprint.radius;
    frame_id_ = msg.name;
    show_label_ = true;
    show_circles_ = false;
    visualization_status_ = 0;
    for(unsigned int i = 0 ; i < msg.robot.laserSensors.size() ; i++)
    {
      CGuiLaser *l = new CGuiLaser(msg.robot.laserSensors[i], frame_id_);
      lasers_.push_back(l);
    }
    for(unsigned int i = 0 ; i < msg.robot.sonarSensors.size() ; i++)
    {
      CGuiSonar *l = new CGuiSonar(msg.robot.sonarSensors[i], frame_id_);
      sonars_.push_back(l);
    }
    for(unsigned int i = 0 ; i < msg.robot.rfidSensors.size() ; i++)
    {
      CGuiRfid *l = new CGuiRfid(msg.robot.rfidSensors[i], frame_id_);
      rfids_.push_back(l);
    }
    for(unsigned int i = 0 ; i < msg.robot.co2Sensors.size() ; i++)
    {
      CGuiCO2 *l = new CGuiCO2(msg.robot.co2Sensors[i], frame_id_);
      co2_sensors_.push_back(l);
    }
    for(unsigned int i = 0 ; i < msg.robot.thermalSensors.size() ; i++)
    {
      CGuiThermal *l = new CGuiThermal(msg.robot.thermalSensors[i], frame_id_);
      thermal_sensors_.push_back(l);
    }
    for(unsigned int i = 0 ; i < msg.robot.soundSensors.size() ; i++)
    {
      CGuiSound *l = new CGuiSound(msg.robot.soundSensors[i], frame_id_);
      sound_sensors_.push_back(l);
    }
    robot_initialized_ = true;
  }
  
  /**
  @brief Callback for the ros laser message
  @param msg [const sensor_msgs::LaserScan&] The new laser scan message
  @return void
  **/
  void CGuiRobot::speedsCallback(const geometry_msgs::Twist& msg)
  {
    linear_speed_ = msg.linear.x;
    linear_speed_y_ = msg.linear.y;
    angular_speed_ = msg.angular.z;
  }
  
  /**
  @brief Paints the robot and it's sensors to the image
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @param listener [tf::TransformListener *] ROS tf listener to get the robot's current pose
  @return void
  **/
  void CGuiRobot::draw(QImage *m,float ocgd,tf::TransformListener *listener)
  {
    if(!robot_initialized_)
    {
      return;
    }
    if(robot_initialized_ && !started_)
    {
      ros::NodeHandle n_;
      std::string speeds_topic = frame_id_ + std::string("/cmd_vel");
      speeds_subscriber_ = n_.subscribe(
        speeds_topic.c_str(), 
        1, 
        &CGuiRobot::speedsCallback,
        this);
    }
    started_ = true;
    resolution_ = ocgd;
    tf::StampedTransform transform;
      
    try
    {
      listener->waitForTransform("map_static",
                                  frame_id_.c_str(),
                                  ros::Time(0),
                                  ros::Duration(0.2));
      listener->lookupTransform("map_static", 
        frame_id_.c_str(), ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_DEBUG("%s",ex.what());
    }
    tfScalar roll,pitch,yaw;
    current_pose_.x = transform.getOrigin().x();
    current_pose_.y = transform.getOrigin().y();
    transform.getBasis().getRPY(roll,pitch,yaw);
    current_pose_.theta = yaw;
    
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      lasers_[i]->paint(m,resolution_,listener);
    }
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      sonars_[i]->paint(m,resolution_,listener);
    }
    for(unsigned int i = 0 ; i < rfids_.size() ; i++)
    {
      rfids_[i]->paint(m,resolution_,listener);
    }
    for(unsigned int i = 0 ; i < co2_sensors_.size() ; i++)
    {
      co2_sensors_[i]->paint(m,resolution_,listener);
    }
    for(unsigned int i = 0 ; i < thermal_sensors_.size() ; i++)
    {
      thermal_sensors_[i]->paint(m,resolution_,listener);
    }
    for(unsigned int i = 0 ; i < sound_sensors_.size() ; i++)
    {
      sound_sensors_[i]->paint(m,resolution_,listener);
    }
    
    drawSelf(m);
  }
  
  /**
  @brief Draws the robot body 
  @param m [QImage*] The image for the robot to draw itself
  @return void
  **/
  void CGuiRobot::drawSelf(QImage *m)
  {
    QPainter painter(m);
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    painter.setPen(QColor(0,0,200,50 + 100 * (2 - visualization_status_)));
    
    if(footprint_.points.size() == 0)
    {
      painter.drawEllipse(
        (current_pose_.x - radius_) / resolution_,
        (current_pose_.y - radius_) / resolution_,
        radius_ * 2.0 / resolution_,
        radius_ * 2.0 / resolution_);
      
      painter.drawLine(	
        current_pose_.x / resolution_,
        current_pose_.y / resolution_,
        current_pose_.x / resolution_ + 
          radius_ / resolution_ * 1.05 * cos(current_pose_.theta),
        current_pose_.y / resolution_ + 
          radius_ / resolution_ * 1.05 * sin(current_pose_.theta));
    }
    else
    {
      float max = -1;
      
      static QPointF *points = new QPointF[footprint_.points.size() + 1];
      
      for(unsigned int i = 0 ; i < footprint_.points.size() + 1; i++)
      {
        
        float x = footprint_.points[i % footprint_.points.size()].x;
        float y = footprint_.points[i % footprint_.points.size()].y;
        
        points[i] = QPointF(
          current_pose_.x / resolution_ + 
            x / resolution_ * cos(- current_pose_.theta) 
            + y / resolution_ * sin(- current_pose_.theta),
              
          current_pose_.y / resolution_ + 
            x / resolution_ * sin(current_pose_.theta) 
            + y / resolution_ * cos(- current_pose_.theta));
    
        if(max < footprint_.points[i].y)
        {
          max = footprint_.points[i].y;
        }
        if(max < footprint_.points[i].x)
        {
          max = footprint_.points[i].x;
        }
      }
      
      painter.drawPolyline(points, footprint_.points.size() + 1);
      
      painter.drawLine(
        QPointF(  current_pose_.x / resolution_,
                  current_pose_.y / resolution_),
        QPointF(  current_pose_.x / resolution_ + 
                    max / resolution_ * 1.05 * cos(current_pose_.theta),
                  current_pose_.y / resolution_ + 
                    max / resolution_ * 1.05 * sin(current_pose_.theta)));
    }
    
    if(show_circles_)
    {
      painter.setPen(QColor(255,0,0,50 + 100 * (2 - visualization_status_)));
      for(unsigned int i = 0 ; i < 5 ; i++)
      {
        painter.drawEllipse(
          (current_pose_.x - (i + 1.0) / 2.0) / resolution_,
          (current_pose_.y - (i + 1.0) / 2.0) / resolution_,
          (i + 1.0) / resolution_,
          (i + 1.0) / resolution_);
      }
    }
  }
  
  /**
  @brief Checks if the robot is near a specific point
  @param p [QPoint] A point
  @return bool : True if the robot is in proximity with p
  **/
  bool CGuiRobot::checkEventProximity(QPoint p)
  {
    float dx = p.x() * resolution_ - current_pose_.x;
    float dy = p.y() * resolution_ - current_pose_.y;
    float dist = sqrt( pow(dx,2) + pow(dy,2) );
    return dist <= 0.3;
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  CGuiRobot::~CGuiRobot(void)
  {
    
  }
  
  /**
  @brief Destroys the robot object
  @return void
  **/
  void CGuiRobot::destroy(void){
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      delete lasers_[i];
    }
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      delete sonars_[i];
    }
  }

  /**
  @brief Returns the frame id of the specific robot
  @return std::string : The robot frame id
  **/
  std::string CGuiRobot::getFrameId(void)
  {
    return frame_id_;
  }
  
  /**
  @brief Draws the robot's label
  @param m [QImage*] The image to be drawn
  @param ocgd [float] The map's resolution
  @return void
  **/
  void CGuiRobot::drawLabel(QImage *m,float ocgd)
  {
    QPainter painter(m);
    
    int text_size = frame_id_.size();
    
    painter.setPen(QColor(0,0,0,100 * (2 - visualization_status_)));
    
    painter.drawRect(
      current_pose_.x / ocgd + 10,
      m->height() - (current_pose_.y / ocgd) - 30,
      3 + text_size * 9,
      20);
    
    painter.setPen(QColor(255,255,255,100 * (2 - visualization_status_)));
    
    painter.fillRect(
      current_pose_.x / ocgd + 10,
      m->height() - (current_pose_.y / ocgd) - 30,
      3 + text_size * 9,
      20,
      QBrush(QColor(0,0,0,100 * (2 - visualization_status_))));
    
    painter.setFont(QFont("Courier New"));
    painter.drawText(
      current_pose_.x / ocgd + 12,
      m->height() - (current_pose_.y / ocgd) - 15,
      QString(frame_id_.c_str()));
  }
  
  /**
  @brief Sets the show_label_ flag
  @param b [bool] True for showing the label
  @return void
  **/
  void CGuiRobot::setShowLabel(bool b)
  {
    show_label_ = b;
  }
  
  /**
  @brief Gets the show_label_ flag
  @return bool : show_label_
  **/
  bool CGuiRobot::getShowLabel(void)
  {
    return show_label_;
  }
  
  /**
  @brief Gets the show_label_ flag
  @return bool : show_label_
  **/
  void CGuiRobot::toggleShowLabel(void)
  {
    show_label_ =! show_label_;
  }
  
  /**
  @brief Toggles the show_circles_ flag
  @return void
  **/
  void CGuiRobot::toggleShowCircles(void)
  {
    show_circles_ =! show_circles_;
  }
  
  /**
  @brief Returns the current robot pose
  @return QPoint : The current robot pose
  **/
  QPoint CGuiRobot::getCurrentPose(void)
  {
    return QPoint(current_pose_.x / resolution_, 
      current_pose_.y / resolution_);
  }
  
  /**
  @brief Returns the current robot pose in meters
  @return geometry_msgs::Pose2D : The current robot pose
  **/
  geometry_msgs::Pose2D CGuiRobot::getCurrentPoseM(void)
  {
    return current_pose_;
  }
  
  /**
  @brief Returns the lasers number
  @return int : the lasers number
  **/
  int CGuiRobot::getLasersNumber(void)
  {
    return lasers_.size();
  }
  
  /**
  @brief Returns the sonars number
  @return int : the sonars number
  **/
  int CGuiRobot::getSonarsNumber(void)
  {
    return sonars_.size();
  }
  
  /**
  @brief Returns the visibility status
  @return char
  **/
  QImage CGuiRobot::getVisualization(float ocgd)
  {
    float maxRange = -1;
    for(unsigned int l = 0 ; l < lasers_.size() ; l++)
    {
      float t = lasers_[l]->getMaxRange();
      if(t > maxRange)
      {
        maxRange = t;
      }
    }
    for(unsigned int l = 0 ; l < sonars_.size() ; l++)
    {
      float t = sonars_[l]->getMaxRange();
      if(t > maxRange)
      {
        maxRange = t;
      }
    }
    visualization = QImage(310,310,QImage::Format_RGB32);
    visualization.fill(Qt::white);
    for(unsigned int l = 0 ; l < lasers_.size() ; l++)
    {
      lasers_[l]->visualizerPaint(&visualization,ocgd,maxRange);
    }
    for(unsigned int l = 0 ; l < sonars_.size() ; l++)
    {
      sonars_[l]->visualizerPaint(&visualization,ocgd,maxRange);
    }
    return visualization;
  }
  
  /**
  @brief Returns the laser visibility status
  @param frame_id [std::string] The laser frame id
  @return char
  **/
  char CGuiRobot::getLaserVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      if(lasers_[i]->getFrameId() == frame_id)
      {
        return lasers_[i]->getVisualizationStatus();
      }
    }
    return 0;
  }
  
  /**
  @brief Returns the rfid reader visibility status
  @param frame_id [std::string] The rfid reader frame id
  @return char
  **/
  char CGuiRobot::getRfidReaderVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < rfids_.size() ; i++)
    {
      if(rfids_[i]->getFrameId() == frame_id)
      {
        return rfids_[i]->getVisualizationStatus();
      }
    }
    return 0;
  }
  
  /**
  @brief Returns the co2 sensor visibility status
  @param frame_id [std::string] The co2 sensor frame id
  @return char
  **/
  char CGuiRobot::getCO2SensorVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < co2_sensors_.size() ; i++)
    {
      if(co2_sensors_[i]->getFrameId() == frame_id)
      {
        return co2_sensors_[i]->getVisualizationStatus();
      }
    }
    return 0;
  }
  
  /**
  @brief Returns the thermal sensor visibility status
  @param frame_id [std::string] The thermal sensor frame id
  @return char
  **/
  char CGuiRobot::getThermalSensorVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < thermal_sensors_.size() ; i++)
    {
      if(thermal_sensors_[i]->getFrameId() == frame_id)
      {
        return thermal_sensors_[i]->getVisualizationStatus();
      }
    }
    return 0;
  }
  
  /**
  @brief Returns the sound sensor visibility status
  @param frame_id [std::string] The sound sensor frame id
  @return char
  **/
  char CGuiRobot::getSoundSensorVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < sound_sensors_.size() ; i++)
    {
      if(sound_sensors_[i]->getFrameId() == frame_id)
      {
        return sound_sensors_[i]->getVisualizationStatus();
      }
    }
    return 0;
  }
    
  /**
  @brief Toggles the laser visibility status
  @param frame_id [std::string] The laser frame id
  @return void
  **/
  void CGuiRobot::toggleLaserVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      if(lasers_[i]->getFrameId() == frame_id)
      {
        lasers_[i]->toggleVisualizationStatus();
      }
    }
  }
  
  /**
  @brief Returns the sonar visibility status
  @param frame_id [std::string] The sonar frame id
  @return char
  **/
  char CGuiRobot::getSonarVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      if(sonars_[i]->getFrameId() == frame_id)
      {
        return sonars_[i]->getVisualizationStatus();
      }
    }
    return 0;
  }
  
  /**
  @brief Toggles the sonar visibility status
  @param frame_id [std::string] The sonar frame id
  @return void
  **/
  void CGuiRobot::toggleSonarVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      if(sonars_[i]->getFrameId() == frame_id)
      {
        sonars_[i]->toggleVisualizationStatus();
      }
    }
  }
  
  /**
  @brief Toggles the rfid reader visibility status
  @param frame_id [std::string] The rfid reader frame id
  @return void
  **/
  void CGuiRobot::toggleRfidReaderVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < rfids_.size() ; i++)
    {
      if(rfids_[i]->getFrameId() == frame_id)
      {
        rfids_[i]->toggleVisualizationStatus();
      }
    }
  }
  /**
  @brief Toggles the co2 sensor visibility status
  @param frame_id [std::string] The co2 sensor frame id
  @return void
  **/
  void CGuiRobot::toggleCO2SensorVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < co2_sensors_.size() ; i++)
    {
      if(co2_sensors_[i]->getFrameId() == frame_id)
      {
        co2_sensors_[i]->toggleVisualizationStatus();
      }
    }
  }
  /**
  @brief Toggles the thermal sensor visibility status
  @param frame_id [std::string] The thermal sensor frame id
  @return void
  **/
  void CGuiRobot::toggleThermalSensorVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < thermal_sensors_.size() ; i++)
    {
      if(thermal_sensors_[i]->getFrameId() == frame_id)
      {
        thermal_sensors_[i]->toggleVisualizationStatus();
      }
    }
  }
  /**
  @brief Toggles the sound sensor visibility status
  @param frame_id [std::string] The sound sensor frame id
  @return void
  **/
  void CGuiRobot::toggleSoundSensorVisualizationStatus(std::string frame_id)
  {
    for(unsigned int i = 0 ; i < sound_sensors_.size() ; i++)
    {
      if(sound_sensors_[i]->getFrameId() == frame_id)
      {
        sound_sensors_[i]->toggleVisualizationStatus();
      }
    }
  }
  
  /**
  @brief Returns the visibility status
  @return char
  **/
  char CGuiRobot::getVisualizationStatus(void)
  {
    return visualization_status_;
  }
  
  /**
  @brief Toggles the visibility status
  @return void
  **/
  void CGuiRobot::toggleVisualizationStatus(void)
  {
    visualization_status_ = (visualization_status_ + 1) % 3;
    for(unsigned int i = 0 ; i < lasers_.size() ; i++)
    {
      lasers_[i]->setVisualizationStatus(visualization_status_);
    }
    for(unsigned int i = 0 ; i < sonars_.size() ; i++)
    {
      sonars_[i]->setVisualizationStatus(visualization_status_);
    }
    for(unsigned int i = 0 ; i < rfids_.size() ; i++)
    {
      rfids_[i]->setVisualizationStatus(visualization_status_);
    }
    for(unsigned int i = 0 ; i < co2_sensors_.size() ; i++)
    {
      co2_sensors_[i]->setVisualizationStatus(visualization_status_);
    }
    for(unsigned int i = 0 ; i < thermal_sensors_.size() ; i++)
    {
      thermal_sensors_[i]->setVisualizationStatus(visualization_status_);
    }
    for(unsigned int i = 0 ; i < sound_sensors_.size() ; i++)
    {
      sound_sensors_[i]->setVisualizationStatus(visualization_status_);
    }
  }
  
  /**
  @brief Returns the current robot speed
  @return std::vector<float> : The linear and angular speeds
  **/
  std::vector<float> CGuiRobot::getSpeeds(void)
  {
    std::vector<float> speeds;
    speeds.push_back(linear_speed_);
    speeds.push_back(linear_speed_y_);
    speeds.push_back(angular_speed_);
    return speeds; 
  }
  
  /**
  @brief Sets the tags existent in the environment
  @param env_tags [stdr_msgs::RfidTagVector] The tag vector
  @return void
  **/
  void CGuiRobot::setEnvironmentalTags(stdr_msgs::RfidTagVector env_tags)
  {
    for(unsigned int i = 0 ; i < rfids_.size() ; i++)
    {
      rfids_[i]->setEnvironmentalTags(env_tags);
    }
  }
  
  /**
  @brief Sets the co2 sources existent in the environment
  **/
  void CGuiRobot::setEnvironmentalCO2Sources(stdr_msgs::CO2SourceVector env_)
  {
    for(unsigned int i = 0 ; i < co2_sensors_.size() ; i++)
    {
      co2_sensors_[i]->setEnvironmentalCO2Sources(env_);
    }
  }
  /**
  @brief Sets the thermal sources existent in the environment
  **/
  void CGuiRobot::setEnvironmentalThermalSources(stdr_msgs::ThermalSourceVector env_)
  {
    for(unsigned int i = 0 ; i < thermal_sensors_.size() ; i++)
    {
      thermal_sensors_[i]->setEnvironmentalThermalSources(env_);
    }
  }
  /**
  @brief Sets the sound sources existent in the environment
  **/
  void CGuiRobot::setEnvironmentalSoundSources(stdr_msgs::SoundSourceVector env_)
  {
    for(unsigned int i = 0 ; i < sound_sensors_.size() ; i++)
    {
      sound_sensors_[i]->setEnvironmentalSoundSources(env_);
    }
  }
}
