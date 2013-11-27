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

#include "stdr_gui/stdr_tools.h"

namespace stdr_gui{
	std::string getRosPackagePath(std::string package){
		return ros::package::getPath(package.c_str());
	}
	
	QString getLiteralTime(int ms){
		QString str;
		int h=ms/(1000*60*60);
		int m=ms/(1000*60)-h*60;
		int s=ms/1000-h*60*60-m*60;
		int ms_=ms-h*60*60*1000-m*1000*60-s*1000;
		if(h)
			str+=QString().setNum(h)+QString(" h ");
		if(m || h)
			str+=QString().setNum(m)+QString(" min ");
		if(s || h || m)
			str+=QString().setNum(s)+QString(" sec ");
		if(ms_ || s || h || m)
			str+=QString().setNum(ms_)+QString(" ms");
		return str;
	}
}
