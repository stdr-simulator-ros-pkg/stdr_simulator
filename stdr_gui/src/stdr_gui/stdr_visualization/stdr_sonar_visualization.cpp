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

#include "stdr_gui/stdr_visualization/stdr_sonar_visualization.h"

namespace stdr_gui{
	SonarVisualisation::SonarVisualisation(QString name){
		this->name=name;
		setupUi(this);
		setWindowTitle(name);
		active=true;
	}
	
	void SonarVisualisation::destruct(void){
		hide();
		delete sonarDistBar;
		delete sonarDist;
		delete sonarMaxDist;
		delete sonarMinDist;
	}
	
	void SonarVisualisation::closeEvent(QCloseEvent *event){
		destruct();
		active=false;
	}
	
	bool SonarVisualisation::getActive(void){
		return active;
	}
	
	void SonarVisualisation::setSonar(stdr_msgs::SonarSensorMsg& msg){
		_msg=msg;
		sonarMaxDist->setText(QString().setNum(msg.maxRange)+QString(" m"));
		sonarMinDist->setText(QString().setNum(msg.minRange)+QString(" m"));
	}
}
