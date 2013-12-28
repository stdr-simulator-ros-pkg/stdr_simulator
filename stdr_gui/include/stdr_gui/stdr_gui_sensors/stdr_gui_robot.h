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

#ifndef STDR_GUI_ROBOT_CONTAINER
#define STDR_GUI_ROBOT_CONTAINER

#include "stdr_gui/stdr_tools.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_laser.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_rfid.h"
#include "stdr_gui/stdr_gui_sensors/stdr_gui_sonar.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CGuiRobot
  @brief Implements the functionalities for a robot
  **/ 
  class CGuiRobot
  {
    //------------------------------------------------------------------------//
    private:
    
      //!< If true the robot's label is visible
      bool show_label_;
      //!< If true the proximity circles are visible
      bool show_circles_;
      //!< True when robot is initialized
      bool robot_initialized_;
      
      //!< Radius of robot if shape is circular
      float radius_;
      //!< Map resolution
      float resolution_;
      
      //!< Robot laser sensors
      std::vector<CGuiLaser*>   lasers_;
      //!< Robot sonar sensors
      std::vector<CGuiSonar*>   sonars_;
      //!< Robot Rfid antenna sensors
      std::vector<CGuiRfid*>     rfids_;
      
      //!< Robot frame id
      std::string frame_id_;
      //!< Initial robot pose
      geometry_msgs::Pose2D initial_pose_;
      //!< Current robot pose
      geometry_msgs::Pose2D current_pose_;
      
      //!< Robot footprint if not circular
      stdr_msgs::FootprintMsg footprint_;
      
      //!< Visualization image
      QImage visualization;
      
      /**
      @brief Draws the robot body 
      @param m [QImage*] The image for the robot to draw itself
      @return void
      **/
      void drawSelf(QImage *m);
      
      //!< The robot visibility status
      char visualization_status_;
    //------------------------------------------------------------------------//
    public:
    
      /**
      @brief Default contructor
      @param msg [const stdr_msgs::RobotIndexedMsg&] The robot description msg
      @return void
      **/
      CGuiRobot(const stdr_msgs::RobotIndexedMsg& msg);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CGuiRobot(void);
      
      /**
      @brief Returns the frame id of the specific robot
      @return std::string : The robot frame id
      **/
      std::string getFrameId(void);
      
      /**
      @brief Paints the robot and it's sensors to the image
      @param m [QImage*] The image to be drawn
      @param ocgd [float] The map's resolution
      @param listener [tf::TransformListener *] ROS tf listener to get the robot's current pose
      @return void
      **/
      void draw(QImage *m,float ocgd,tf::TransformListener *listener);
      
      /**
      @brief Draws the robot's label
      @param m [QImage*] The image to be drawn
      @param ocgd [float] The map's resolution
      @return void
      **/
      void drawLabel(QImage *m,float ocgd);
      
      /**
      @brief Checks if the robot is near a specific point
      @param p [QPoint] A point
      @return bool : True if the robot is in proximity with p
      **/
      bool checkEventProximity(QPoint p);
      
      /**
      @brief Sets the show_label_ flag
      @param b [bool] True for showing the label
      @return void
      **/
      void setShowLabel(bool b);
      
      /**
      @brief Toggles the show_label_ flag
      @return void
      **/
      void toggleShowLabel(void);
      
      /**
      @brief Gets the show_label_ flag
      @return bool : show_label_
      **/
      bool getShowLabel(void);
      
      /**
      @brief Toggles the show_circles_ flag
      @return void
      **/
      void toggleShowCircles(void);
      
      /**
      @brief Returns the current robot pose
      @return QPoint : The current robot pose
      **/
      QPoint getCurrentPose(void);
      
      /**
      @brief Destroys the robot object
      @return void
      **/
      void destroy(void);
      
      /**
      @brief Returns the lasers number
      @return int : the lasers number
      **/
      int getLasersNumber(void);
      
      /**
      @brief Returns the sonars number
      @return int : the sonars number
      **/
      int getSonarsNumber(void);
      
      /**
      @brief Returns the visibility status
      @return char
      **/
      char getVisualizationStatus(void);
      
      /**
      @brief Toggles the visibility status
      @return void
      **/
      void toggleVisualizationStatus(void);
      
      /**
      @brief Returns the laser visibility status
      @param frame_id [std::string] The laser frame id
      @return char
      **/
      char getLaserVisualizationStatus(std::string frame_id);
      
      /**
      @brief Toggles the laser visibility status
      @param frame_id [std::string] The laser frame id
      @return void
      **/
      void toggleLaserVisualizationStatus(std::string frame_id);
      
      /**
      @brief Returns the sonar visibility status
      @param frame_id [std::string] The sonar frame id
      @return char
      **/
      char getSonarVisualizationStatus(std::string frame_id);
      
      /**
      @brief Toggles the sonar visibility status
      @param frame_id [std::string] The sonar frame id
      @return void
      **/
      void toggleSonarVisualizationStatus(std::string frame_id);
      
      /**
      @brief Returns the visualization image of the robot
      @param ocgd [float] The map resolution
      @return QImage : The drawn image
      **/
      QImage getVisualization(float ocgd);
  };  
}

#endif
