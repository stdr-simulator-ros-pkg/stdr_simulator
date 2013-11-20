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
#include "stdr_tools.h"

namespace stdr{
	class MapLoader : public QWidget, public Ui_mapWidget{
			int argc;
			char **argv;
			QImage *internalImg;
			std::pair<int,int> checkDimensions(int w,int h);
		public:
			MapLoader(int argc, char **argv);
			void resizeEvent(QResizeEvent *e);
			
			void updateImage(QImage *img);
	};	
}

#endif
