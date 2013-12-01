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

namespace stdr_gui{
	
	enum StdrMapState{
		NORMAL,
		ZOOMIN,
		ZOOMOUT,
		SETPLACE
	};
	
	class MapConnector : public QObject{
		Q_OBJECT
			int 		_argc;
			char** 		_argv;
			QCursor 	_zoomInCursor;
			QCursor 	_zoomOutCursor;
			
			StdrMapState 	_mapState;
			MapLoader 		_loader;
		
		public:
			MapConnector(int argc, char **argv);
			void updateImage(QImage *img);	
			void setupLoaderToGrid(QGridLayout *layout,int row,int column);
			void setInitialImageSize(QSize s);
			void updateZoom(QPoint p,bool z);
			QPoint getGlobalPoint(QPoint p);
			void drawGrid(QImage *img,float resolution);
			QPoint mapToGlobal(QPoint p);
			
		public Q_SLOTS:
			bool eventFilter( QObject* watched, QEvent* event);
			void serveImage(QImage *img);
			void setCursorZoomIn(bool state);
			void setCursorZoomOut(bool state);
			void setCursorAdjusted(bool state);
			void waitForPlace(void);
		
		Q_SIGNALS:
			void signalUpdateImage(QImage *img);	
			void zoomInPressed(QPoint p);	
			void zoomOutPressed(QPoint p);	
			void robotPlaceSet(QPoint p);
			void itemClicked(QPoint p,Qt::MouseButton b);
	};	
}

#endif
