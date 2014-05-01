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

#ifndef STDR_MAP_CONNECTOR
#define STDR_MAP_CONNECTOR

#include "stdr_gui/stdr_map_loader.h"

/**
@namespace stdr_gui
@brief The main namespace for STDR GUI
**/ 
namespace stdr_gui
{
  /**
  @enum EStdrMapState
  @brief Holds the possible map states
  **/ 
  enum EStdrMapState
  {
    NORMAL,
    ZOOMIN,
    ZOOMOUT,
    SETPLACE,
    SETREPLACE,
    SETPLACECO2,
    SETPLACESOUND,
    SETPLACERFID,
    SETPLACETHERMAL
  };
  
  
  /**
  @class CMapConnector
  @brief Serves the Qt events of the map-holding widget. Inherits from QObject
  **/ 
  class CMapConnector : 
    public QObject
  {
    Q_OBJECT
    //------------------------------------------------------------------------//
    private:
    
      //!< Number of input arguments
      int argc_;
      //!< Input arguments
      char** argv_;
      
      //!< Holds the currently clicked robot frame id
      std::string current_robot_frame_id_;
      
      //!< Mouse cursor for zoom in
      QCursor zoom_in_cursor_;
      //!< Mouse cursor for zoom out
      QCursor zoom_out_cursor_;
      
      //!< Holds the map state
      EStdrMapState map_state_;
      
      //!< Object of CMapLoader
      CMapLoader loader_;
      
      //!< True if map is initialized
      bool map_initialized_;
    
    //------------------------------------------------------------------------//
    public:
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CMapConnector(int argc, char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CMapConnector(void);
      
      /**
      @brief Emits the signalUpdateImage signal
      @param img [QImage*] The image to be updated
      @return void
      **/
      void updateImage(QImage *img);  
      
      /**
      @brief Sets map initial size to the loader
      @param s [QSize] Map size
      @return void
      **/
      void setInitialImageSize(QSize s);
      
      /**
      @brief Updates the map zoom. Wrapper for a loader function
      @param p [QPoint] The point of the zoom event
      @param z [bool] True if zoom in, false if zoom out
      @return void
      **/
      void updateZoom(QPoint p,bool z);
      
      /**
      @brief Updates the map center when a robot is followed
      @param p [QPoint] The new center
      @return void
      **/
      void updateCenter(QPoint p);
      
      /**
      @brief Returns the point in the real map image. Wrapper for a loader function.
      @param p [QPoint] A point
      @return QPoint : The "real" point in the original map
      **/
      QPoint getGlobalPoint(QPoint p);
      
      /**
      @brief Wrapper for the draw grid function of loader
      @param img [QImage*] The image on which the grid will be painted
      @param resolution [float] The map resolution
      @return void
      **/
      void drawGrid(QImage *img,float resolution);
      
      /**
      @brief Calls the Qt function that gets the real point that the event happened
      @param p [QPoint] The point of the event
      @return QPoint : The "real" point
      **/
      QPoint mapToGlobal(QPoint p);
      
      /**
      @brief Returns the CMapLoader object
      @return QWidget* : The object
      **/
      QWidget* getLoader(void);
      
      /**
      @brief Sets the map initialization status
      @param mi [bool] The initialization status
      @return void
      **/
      void setMapInitialized(bool mi);
    
    //------------------------------------------------------------------------//
    public Q_SLOTS:
    
      /**
      @brief General event filter. Captures all events
      @param watched [QObject*] The object in which the event was triggered
      @param event [QEvent*] The type of event
      @return bool : True is event served
      **/
      bool eventFilter( QObject* watched, QEvent* event);
      
      /**
      @brief Called from signalUpdateImage signal. Calls the updateImage of CMapLoader
      @param img [QImage*] The image to be painted
      @return void
      **/
      void serveImage(QImage *img);
      
      /**
      @brief Called when zoom in event happens
      @param state [bool] True when zoom in active
      @return void
      **/
      void setCursorZoomIn(bool state);
      
      /**
      @brief Called when zoom out event happens
      @param state [bool] True when zoom out active
      @return void
      **/
      void setCursorZoomOut(bool state);
      
      /**
      @brief Called when zoom adjusted event happens
      @param state [bool] True when zoom adjusted active
      @return void
      **/
      void setCursorAdjusted(bool state);
      
      /**
      @brief Changes the map state. Waits for a robot to be placed
      @return void
      **/
      void waitForPlace(void);
      
      /**
      @brief Changes the map state. Waits for a thermal source to be placed
      @return void
      **/
      void waitForThermalPlace(void);
      
      /**
      @brief Changes the map state. Waits for a CO2 source to be placed
      @return void
      **/
      void waitForCo2Place(void);
      
      /**
      @brief Changes the map state. Waits for a sound source to be placed
      @return void
      **/
      void waitForSoundPlace(void);
      
      /**
      @brief Changes the map state. Waits for an RFID tag to be placed
      @return void
      **/
      void waitForRfidPlace(void);
      
      /**
      @brief Changes the map state. Waits for a robot to be re-placed
      @param robotFrameId [std::string] The robot frame id of robot to be re-placed
      @return void
      **/
      void waitForReplace(std::string robotFrameId);
    
    //------------------------------------------------------------------------//
    Q_SIGNALS:
    
      /**
      @brief Emmited in updateImage function
      @param img [QImage*] The image to be updated
      @return void
      **/
      void signalUpdateImage(QImage *img);  
      
      /**
      @brief Emmited when click is captured and state is ZOOMIN
      @param p [QPoint] The event point
      @return void
      **/
      void zoomInPressed(QPoint p);  
      
      /**
      @brief Emmited when click is captured and state is ZOOMOUT
      @param p [QPoint] The event point
      @return void
      **/
      void zoomOutPressed(QPoint p);  
      
      /**
      @brief Emmited when click is captured and state is SETPLACE
      @param p [QPoint] The event point
      @return void
      **/
      void robotPlaceSet(QPoint p);
      
      /**
      @brief Emmited when click is captured and state is SETTHERMALPLACE
      @param p [QPoint] The event point
      @return void
      **/
      void thermalPlaceSet(QPoint p);
      
      /**
      @brief Emmited when click is captured and state is SETCO2PLACE
      @param p [QPoint] The event point
      @return void
      **/
      void co2PlaceSet(QPoint p);
      
      /**
      @brief Emmited when click is captured and state is SETSOUNDPLACE
      @param p [QPoint] The event point
      @return void
      **/
      void soundPlaceSet(QPoint p);
      
      /**
      @brief Emmited when click is captured and state is SETRFIDPLACE
      @param p [QPoint] The event point
      @return void
      **/
      void rfidPlaceSet(QPoint p);
      
      /**
      @brief Emmited when click is captured and state is NORMAL
      @param p [QPoint] The event point
      @param b [Qt::MouseButton] The mouse button clicked
      @return void
      **/
      void itemClicked(QPoint p,Qt::MouseButton b);
      
      /**
      @brief Emmited when click is captured and state is SETREPLACE
      @param p [QPoint] The event point
      @param robotName [std::string] Frame id of the robot re-placed
      @return void
      **/
      void robotReplaceSet(QPoint p,std::string robotName);
  };  
}

#endif
