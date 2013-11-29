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

#include "stdr_gui/stdr_visualization/stdr_laser_visualization.h"

namespace stdr_gui{
	LaserVisualisation::LaserVisualisation(QString name){
		this->name=name;
		setupUi(this);
		setWindowTitle(name);
		active=true;
	}
	
	void LaserVisualisation::destruct(void){
		hide();
		delete laserMean;
		delete laserMax;
		delete laserMin;
		delete laserImage;
	}
	
	void LaserVisualisation::closeEvent(QCloseEvent *event){
		destruct();
		active=false;
	}
	
	bool LaserVisualisation::getActive(void){
		return active;
	}
	
	void LaserVisualisation::setLaser(stdr_msgs::LaserSensorMsg& msg){
		_msg=msg;
		laserMax->setText(QString().setNum(msg.maxRange)+QString(" m"));
		laserMin->setText(QString().setNum(msg.minRange)+QString(" m"));
	}
}