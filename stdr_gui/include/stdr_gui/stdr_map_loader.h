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

namespace stdr_gui{
	class CMapLoader : 
		public QWidget, 
		public Ui_mapWidget
	{
		private:
			int 	argc_;
			char**	argv_;
			int 	zoom_;
			
			QImage*	internal_img_;
			QPoint map_min_;
			QPoint map_max_;
			QSize initial_image_size_;
			
			QPoint pointUnscaled(QPoint p);
			std::pair<int,int> checkDimensions(int w,int h);
			
		public:
			
			CMapLoader(int argc, char **argv);
			
			void setInitialImageSize(QSize s);
			void resizeEvent(QResizeEvent *e);
			void updateImage(QImage *img);
			void drawGrid(QImage *img,float resolution);
			void updateZoom(QPoint p,bool zoomIn);
			void updateCenter(QPoint p);
			void resetZoom(void);
			QPoint getGlobalPoint(QPoint);
			void wheelEvent ( QWheelEvent * event );
	};	
}

#endif
