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

#ifndef STDR_INFO_LOADER
#define STDR_INFO_LOADER

#include "ui_information.h"
#include "stdr_gui/stdr_tools.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CInfoLoader
  @brief Implements the low level Qt functionalities of the information tree. Inherits from Ui_information (generated from an ui file) and QWidget. 
  **/ 
  class CInfoLoader : 
    public QWidget, 
    public Ui_information
  {
    //------------------------------------------------------------------------//
    private:
    
      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char **  argv_;
        
      //!< Icon of visualization actions
      QIcon visible_icon_;
      
    //------------------------------------------------------------------------//
    public:
    
      //!< Icon of visibility on (green)
      QIcon visible_icon_on_;
      
      //!< Icon of visibility off (red)
      QIcon visible_icon_off_;
      
      //!< Icon of visibility medium (orange)
      QIcon visible_icon_trans_;
      
      //!< Tree item : Root
      QTreeWidgetItem generalInfo;
      //!< Tree item : Robot root
      QTreeWidgetItem robotsInfo;
      //!< Tree item : Map height
      QTreeWidgetItem mapHeight;
      //!< Tree item : Map width
      QTreeWidgetItem mapWidth;
      //!< Tree item : Map resolution
      QTreeWidgetItem mapOcgd;
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CInfoLoader(int argc, char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CInfoLoader(void);
      
      /**
      @brief Deletes the information tree
      @return void
      **/
      void deleteTree(void);
      
      /**
      @brief Deletes a specific tree node. Recursive function.
      @param item [QTreeWidgetItem*] The item to be deleted
      @return void
      **/
      void deleteTreeNode(QTreeWidgetItem *item);
      
      /**
      @brief Updates the information tree according to the specific map
      @param width [float] The map width
      @param height [float] The map height
      @param ocgd [float] The map resolution (m/pixel)
      @return void
      **/
      void updateMapInfo(float width,float height,float ocgd);
      
      /**
      @brief Updates the information tree according to the ensemble of robots
      @param msg [const stdr_msgs::RobotIndexedVectorMsg&] The existent robots
      @return void
      **/
      void updateRobots(const stdr_msgs::RobotIndexedVectorMsg& msg);
      
      /**
      @brief Autoresizes the columns according to the visible contents
      @return void
      **/
      void autoResizeColumns(void);
  };  
}

#endif
