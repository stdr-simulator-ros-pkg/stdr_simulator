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

#ifndef STDR_ROBOT_VISUALIZATION
#define STDR_ROBOT_VISUALIZATION

#include "stdr_gui/stdr_tools.h"
#include "ui_robotVisualization.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CRobotVisualisation
  @brief Implements the functionalities of the robot visualization widget. Inherits form QWidget and Ui_robotVisualization (auto created from ui file)
  **/ 
  class CRobotVisualisation : 
    public QWidget, 
    public Ui_robotVisualization
  {
    //------------------------------------------------------------------------//
    private:
      //!< True if the visualizer is active
      bool active_;
      //!< The map resolution
      float resolution_;
      //!< The image to draw into
      QImage internal_image_;
      //!< A void image
      QImage void_image_;
      //!< The robot frame id
      QString name_;
    //------------------------------------------------------------------------//  
    public:
      /**
      @brief Default contructor
      @param name [QString] Robot frame id
      @param resolution [float] Map resolution
      @return void
      **/
      CRobotVisualisation(QString name,float resolution);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CRobotVisualisation(void);
    
      /**
      @brief Returns true if the visualizer is active
      @return bool
      **/
      bool getActive(void);
      
      /**
      @brief Destroys the visualizer
      @return void
      **/
      void destruct(void);
      
      /**
      @brief Called when the close event is triggered
      @param event [QCloseEvent*] The close event
      @return void
      **/
      void closeEvent(QCloseEvent *event);
      
      /**
      @brief Returns the internal image size
      @return int : Width of the internal image
      **/
      int getInternalImageSize(void);
      
      /**
      @brief Sets the image to be shown
      @param img [QImage] The drawn image
      @return void
      **/
      void setImage(QImage img);
      
      /**
      @brief Sets the robot's current pose
      @param pose [geometry_msgs::Pose2D] the robot pose
      @return void
      **/
      void setCurrentPose(geometry_msgs::Pose2D pose);
      
      /**
      @brief Sets the robot's current speed
      @param msg [std::vector<float>] the robot speeds
      @return void
      **/
      void setCurrentSpeed(std::vector<float> msg);
  };  
}

#endif
