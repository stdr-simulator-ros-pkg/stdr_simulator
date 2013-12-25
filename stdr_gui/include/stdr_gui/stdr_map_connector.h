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

namespace stdr_gui
{
  
  enum EStdrMapState
  {
    NORMAL,
    ZOOMIN,
    ZOOMOUT,
    SETPLACE,
    SETREPLACE,
    SETPLACECO2,
    SETPLACERFID,
    SETPLACETHERMAL
  };
  
  class CMapConnector : 
    public QObject
  {
    Q_OBJECT
      int     argc_;
      char**     argv_;
      
      std::string   current_robot_frame_id_;
      
      QCursor   zoom_in_cursor_;
      QCursor   zoom_out_cursor_;
      
      EStdrMapState   map_state_;
      CMapLoader     loader_;
      
      bool map_initialized_;
      
    public:
      CMapConnector(int argc, char **argv);
      ~CMapConnector(void);
      
      void updateImage(QImage *img);  
      void setInitialImageSize(QSize s);
      void updateZoom(QPoint p,bool z);
      void updateCenter(QPoint p);
      QPoint getGlobalPoint(QPoint p);
      void drawGrid(QImage *img,float resolution);
      QPoint mapToGlobal(QPoint p);
      QWidget* getLoader(void);
      void setMapInitialized(bool mi);
      
    public Q_SLOTS:
      bool eventFilter( QObject* watched, QEvent* event);
      void serveImage(QImage *img);
      void setCursorZoomIn(bool state);
      void setCursorZoomOut(bool state);
      void setCursorAdjusted(bool state);
      void waitForPlace(void);
      void waitForThermalPlace(void);
      void waitForCo2Place(void);
      void waitForRfidPlace(void);
      void waitForReplace(std::string robotFrameId);
    
    Q_SIGNALS:
      void signalUpdateImage(QImage *img);  
      void zoomInPressed(QPoint p);  
      void zoomOutPressed(QPoint p);  
      void robotPlaceSet(QPoint p);
      void thermalPlaceSet(QPoint p);
      void co2PlaceSet(QPoint p);
      void rfidPlaceSet(QPoint p);
      void itemClicked(QPoint p,Qt::MouseButton b);
      void robotReplaceSet(QPoint p,std::string robotName);
  };  
}

#endif
