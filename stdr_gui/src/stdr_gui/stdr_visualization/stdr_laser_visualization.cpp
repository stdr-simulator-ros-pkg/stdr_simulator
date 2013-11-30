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
	LaserVisualisation::LaserVisualisation(QString name,float resolution){
		this->name=name;
		setupUi(this);
		setWindowTitle(name);
		active=true;
		_resolution=resolution;
		ros::NodeHandle _n;
		_subscriber = _n.subscribe(name.toStdString().c_str(), 1, &LaserVisualisation::callback,this);
		voidImage=QImage(laserImage->width(),laserImage->height(),QImage::Format_RGB32);
		voidImage.fill(QColor(255,255,255,255));
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
		_subscriber.shutdown();
	}
	
	bool LaserVisualisation::getActive(void){
		return active;
	}
	
	void LaserVisualisation::setLaser(stdr_msgs::LaserSensorMsg& msg){
		_msg=msg;
		laserMax->setText(QString().setNum(msg.maxRange)+QString(" m"));
		laserMin->setText(QString().setNum(msg.minRange)+QString(" m"));
	}
	
	void LaserVisualisation::callback(const sensor_msgs::LaserScan& msg){
		scan=msg;
	}
	
	void LaserVisualisation::paint(void){
		internalImage=voidImage;
		QPainter painter(&internalImage);
		painter.setPen(QColor(255,0,0,255));
		float mean=0;
		for(unsigned int i=0;i<scan.ranges.size();i++){
			mean+=scan.ranges[i];
			painter.drawLine(
				internalImage.width()/2,
				internalImage.height()/2,
				internalImage.width()/2+
					scan.ranges[i]/_msg.maxRange*cos(scan.angle_min+((float)i)*scan.angle_increment)*internalImage.width()/2,
				internalImage.height()/2+
					scan.ranges[i]/_msg.maxRange*sin(scan.angle_min+((float)i)*scan.angle_increment)*internalImage.width()/2
			);				
		}
		laserMean->setText(QString().setNum(mean/scan.ranges.size())+QString(" m"));
		laserImage->setPixmap(QPixmap().fromImage(internalImage.mirrored(false,true)));
	}
}
