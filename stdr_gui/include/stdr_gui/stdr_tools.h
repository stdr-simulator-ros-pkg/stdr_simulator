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

#ifndef STDR_TOOLS
#define STDR_TOOLS

#include <ros/package.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <QtUiTools/QUiLoader>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QTime>

#include <QtGui/QMenu>
#include <QtGui/QApplication>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QImage>
#include <QtGui/QFileDialog>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPainter>
#include <QtGui/QPixmap>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollBar>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QTreeWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <QtGui/QWindowsStyle>
#include <QtGui/QInputDialog>
#include <QtGui/QMouseEvent>
#include <QtGui/QMessageBox>
#include <QtGui/QTimeEdit>
#include <QtGui/QInputDialog>
#include <QtGui/QFont>

#include <stdr_msgs/RobotIndexedVectorMsg.h>
#include <stdr_msgs/RobotIndexedMsg.h>
#include <stdr_msgs/RobotMsg.h>
#include <stdr_msgs/Noise.h>

#include <stdr_msgs/RfidTagVector.h>
#include <stdr_msgs/AddRfidTag.h>
#include <stdr_msgs/DeleteRfidTag.h>

#include <stdr_msgs/CO2SourceVector.h>
#include <stdr_msgs/AddCO2Source.h>
#include <stdr_msgs/DeleteCO2Source.h>

#include <stdr_msgs/ThermalSourceVector.h>
#include <stdr_msgs/AddThermalSource.h>
#include <stdr_msgs/DeleteThermalSource.h>

#include <stdr_msgs/SoundSourceVector.h>
#include <stdr_msgs/AddSoundSource.h>
#include <stdr_msgs/DeleteSoundSource.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/LaserScan.h>

#include "stdr_server/map_loader.h"
#include <stdr_msgs/LoadExternalMap.h>

#include <stdr_parser/stdr_parser.h>

#define STDR_PI 3.14159265359

/**
@namespace stdr_gui_tools
@brief The namespace for STDR GUI tools
**/ 
namespace stdr_gui_tools
{
  /**
  @brief Returns the global path of the ROS package provided
  @param package [std::string] The ROS package
  @return std::string : The global path of the specific package
  **/
  std::string getRosPackagePath(std::string package);
  
  /**
  @brief Transforms the milliseconds in literal representation
  @param ms [int] The time in ms
  @return QString : The literal representation of time given
  **/
  QString getLiteralTime(int ms);
  
  /**
  @brief Converts an angle from rads to degrees
  @param angle [float] An angle in rads
  @return float : The angle in degrees
  **/
  float angleRadToDegrees(float angle);
  
  /**
  @brief Converts an angle from degrees to rads
  @param angle [float] An angle in degrees
  @return float : The angle in rads
  **/
  float angleDegreesToRad(float angle);
  
  /**
  @brief Prints a sonar msg
  @param msg [stdr_msgs::SonarSensorMsg &] The message
  @return void
  **/
  void printSonarMsg(stdr_msgs::SonarSensorMsg &msg);
  
  /**
  @brief Prints a laser msg
  @param msg [stdr_msgs::LaserSensorMsg &] The message
  @return void
  **/
  void printLaserMsg(stdr_msgs::LaserSensorMsg &msg);
  
  /**
  @brief Prints a ROS pose2d msg
  @param msg [geometry_msgs::Pose2D &] The message
  @return void
  **/
  void printPose2D(geometry_msgs::Pose2D &msg);
  
  /**
  @brief Takes a stdr_msgs::RobotMsg and converts its angles to rads
  @param rmsg [stdr_msgs::RobotMsg] The robot message
  @return stdr_msgs::RobotMsg : The recreated robot message
  **/
  stdr_msgs::RobotMsg fixRobotAnglesToRad(stdr_msgs::RobotMsg rmsg);
  
  /**
  @brief Takes a stdr_msgs::RobotMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::RobotMsg] The robot message
  @return stdr_msgs::RobotMsg : The recreated robot message
  **/
  stdr_msgs::RobotMsg fixRobotAnglesToDegrees(stdr_msgs::RobotMsg rmsg);
  
  /**
  @brief Takes a stdr_msgs::LaserSensorMsg and converts its angles to rads
  @param rmsg [stdr_msgs::LaserSensorMsg] The laser message
  @return stdr_msgs::LaserSensorMsg : The recreated laser message
  **/
  stdr_msgs::LaserSensorMsg fixLaserAnglesToRad(stdr_msgs::LaserSensorMsg rmsg);
  
  /**
  @brief Takes a stdr_msgs::LaserSensorMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::LaserSensorMsg] The laser message
  @return stdr_msgs::LaserSensorMsg : The recreated laser message
  **/
  stdr_msgs::LaserSensorMsg fixLaserAnglesToDegrees(
    stdr_msgs::LaserSensorMsg rmsg);
    
  /**
  @brief Takes a stdr_msgs::SonarSensorMsg and converts its angles to rads
  @param rmsg [stdr_msgs::SonarSensorMsg] The sonar message
  @return stdr_msgs::SonarSensorMsg : The recreated sonar message
  **/
  stdr_msgs::SonarSensorMsg fixSonarAnglesToRad(stdr_msgs::SonarSensorMsg rmsg);
  
  /**
  @brief Takes a stdr_msgs::SonarSensorMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::SonarSensorMsg] The sonar message
  @return stdr_msgs::SonarSensorMsg : The recreated sonar message
  **/
  stdr_msgs::SonarSensorMsg fixSonarAnglesToDegrees(
    stdr_msgs::SonarSensorMsg rmsg);
    
  /**
  @brief Takes a stdr_msgs::RfidSensorMsg and converts its angles to rads
  @param rmsg [stdr_msgs::RfidSensorMsg] The rfid reader message
  @return stdr_msgs::RfidSensorMsg : The recreated rfid reader message
  **/
  stdr_msgs::RfidSensorMsg fixRfidAnglesToRad(stdr_msgs::RfidSensorMsg rmsg);
  
  /**
  @brief Takes a stdr_msgs::RfidSensorMsg and converts its angles to degrees
  @param rmsg [stdr_msgs::RfidSensorMsg] The rfid reader message
  @return stdr_msgs::RfidSensorMsg : The recreated rfid reader message
  **/
  stdr_msgs::RfidSensorMsg fixRfidAnglesToDegrees(
    stdr_msgs::RfidSensorMsg rmsg);
    
  /**
  @brief Angle conversion
  **/
  stdr_msgs::CO2SensorMsg fixCO2AnglesToRad(stdr_msgs::CO2SensorMsg rmsg);
  
  /**
  @brief Angle conversion
  **/
  stdr_msgs::CO2SensorMsg fixCO2AnglesToDegrees(
    stdr_msgs::CO2SensorMsg rmsg);
    
  /**
  @brief Angle conversion
  **/
  stdr_msgs::ThermalSensorMsg fixThermalAnglesToRad(stdr_msgs::ThermalSensorMsg rmsg);
  
  /**
  @brief Angle conversion
  **/
  stdr_msgs::ThermalSensorMsg fixThermalAnglesToDegrees(
    stdr_msgs::ThermalSensorMsg rmsg);
    
  /**
  @brief Angle conversion
  **/
  stdr_msgs::SoundSensorMsg fixSoundAnglesToRad(stdr_msgs::SoundSensorMsg rmsg);
  
  /**
  @brief Angle conversion
  **/
  stdr_msgs::SoundSensorMsg fixSoundAnglesToDegrees(
    stdr_msgs::SoundSensorMsg rmsg);
}

#endif
