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
	
#include "stdr_gui/stdr_map_metainformation/stdr_gui_co2_source.h"

namespace stdr_gui{
	CGuiCo2Source::CGuiCo2Source(QPoint p):
		position_(p)
	{

	}
	
	CGuiCo2Source::~CGuiCo2Source(void)
	{

	}
	
	std::string CGuiCo2Source::getName(void)
	{
		return name_;
	}
	
	bool CGuiCo2Source::checkProximity(QPoint p){
		return false;	// 2b changed
	}
}

