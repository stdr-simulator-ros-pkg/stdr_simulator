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

#ifndef STDR_MAP_LOADER
#define STDR_MAP_LOADER

#include "ui_map.h"
#include "stdr_gui/stdr_tools.h"

#define ZOOM_RATIO 1.1

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @class CMapLoader
  @brief Implements the low level Qt functionalities of the map widget. Inherits from Ui_mapWidget (generated from an ui file) and QWidget. 
  **/ 
  class CMapLoader : 
    public QWidget, 
    public Ui_mapWidget
  {
    //------------------------------------------------------------------------//
    private:
      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char**  argv_;
      //!< Current zoom. Calculated as pow(2,zoom_) * initial image size
      int   zoom_;
      
      //!< Internal image used before a map is loaded
      QImage*  internal_img_;
      
      //!< The upper left point of map visualization
      QPoint map_min_;
      
      //!< The lower right point of map visualization
      QPoint map_max_;
      
      //!< The original image size
      QSize initial_image_size_;
      
      /**
      @brief Unscales the input point
      @param p [QPoint] Point of an event in the adjusted map
      @return QPoint : The same point in the original map
      **/
      QPoint pointUnscaled(QPoint p);
      
      /**
      @brief Return the dimensions according to the container size
      @param w [int] Image width
      @param h [int] Image height
      @return std::pair<int,int> : The size the map must be resized to
      **/
      std::pair<int,int> checkDimensions(int w,int h);
      
    //------------------------------------------------------------------------//  
    public:
      
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CMapLoader(int argc, char **argv);
      
      /**
      @brief Sets the initial image size
      @param s [QSize] The initial image size
      @return void
      **/
      void setInitialImageSize(QSize s);
      
      /**
      @brief Captures the resize event
      @param e [QResizeEvent*] The resize event
      @return void
      **/
      void resizeEvent(QResizeEvent *e);
      
      /**
      @brief Updates the image
      @param img [QImage*] The image to be updated
      @return void
      **/
      void updateImage(QImage *img);
      
      /**
      @brief Draws a grid in an image
      @param img [QImage*] The image for the grid to be drawn on
      @param resolution [float] The map resolution
      @return void
      **/
      void drawGrid(QImage *img,float resolution);
      
      /**
      @brief Updates the zoom of the image
      @param p [QPoint] The point of the zoom event
      @param zoomIn [bool] True if zoom in, false if zoom out
      @return void
      **/
      void updateZoom(QPoint p,bool zoomIn);
      
      /**
      @brief Updates the image center
      @param p [QPoint] The new center
      @return void
      **/
      void updateCenter(QPoint p);
      
      /**
      @brief Updates the image center by moving directionally
      @param key [int] The key pressed
      @return void
      **/
      void moveDirectionally(int key);
      
      /**
      @brief Resets the zoom of the image
      @return void
      **/
      void resetZoom(void);
      
      /**
      @brief Calculates the "real" point in the image
      @param p [QPoint] The point to be translated
      @return QPoint : The new point
      **/
      QPoint getGlobalPoint(QPoint p);

  };  
}

#endif
