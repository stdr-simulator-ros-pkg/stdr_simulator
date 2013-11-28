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

#include "stdr_gui/stdr_map_loader.h"

namespace stdr_gui{
	MapLoader::MapLoader(int argc, char **argv){
		this->argc=argc;
		this->argv=argv;
		
		setupUi(this);
		internalImg=new QImage(100,100,QImage::Format_RGB32);
		mapMin=QPoint(0,0);
		zoom=0;
	}
	
	void MapLoader::resizeEvent(QResizeEvent *e){
		updateImage(internalImg);
	}
	
	std::pair<int,int> MapLoader::checkDimensions(int w,int h){
		float containerWidth=this->width();
		float containerHeight=this->height();
		float aspectRatio=(float)w/(float)h;
		float finalW,finalH;
		if(containerHeight*aspectRatio>containerWidth){
			finalW=containerWidth;
			finalH=containerWidth/aspectRatio;
		}
		else{
			finalW=containerHeight*aspectRatio;
			finalH=containerHeight;
		}
		return std::pair<int,int>(finalW,finalH);
	}
	
	void MapLoader::updateImage(QImage *img){
		internalImg=img;
		std::pair<int,int> newDims=checkDimensions(img->width(),img->height());
		
		map->setPixmap(
			QPixmap().fromImage(
				(*img).
					copy(	mapMin.x(),
							mapMin.y(),
							mapMax.x()-mapMin.x(),
							mapMax.y()-mapMin.y()).
					scaled(	newDims.first,newDims.second,
							Qt::IgnoreAspectRatio,
							Qt::SmoothTransformation)));
					
		map->resize(newDims.first,newDims.second);
	}
	
	void MapLoader::drawGrid(QImage *img,float resolution){
		QPainter painter(img);
		painter.setPen(QColor(100,100,100,150));
		int pix=1.0/resolution;
		for(unsigned int i=1;i<=img->width()/pix+1;i++)
			painter.drawLine(	0,
								i*pix,
								img->width()-1,
								i*pix);
		for(unsigned int i=1;i<=img->height()/pix+1;i++)
			painter.drawLine(	i*pix,
								0,
								i*pix,
								img->height()-1);
	}
	
	void MapLoader::updateZoom(QPoint p,bool zoomIn){
		ROS_ERROR("Update zoom - point : %d %d",p.x(),p.y());
		QPoint np=getGlobalPoint(p);
		np.setY(internalImg->height()-np.y());	// I have the global point from UP-LEFT
		ROS_ERROR("Update zoom - global point : %d %d",np.x(),np.y());
		int prevZoom=zoom;
		if(zoomIn)	zoom++;
		else zoom--;
		if(zoom<0){
			zoom=0;
			return;
		}
		float intW=internalImg->width();
		float intH=internalImg->height();
		float newWidth=internalImg->width()/pow(2,zoom);
		float newHeight=internalImg->height()/pow(2,zoom);
		QPoint evOriginal=np;
		
		float xmin,xmax,ymin,ymax;
		xmin=evOriginal.x()-newWidth/2;
		xmax=evOriginal.x()+newWidth/2;
		ymin=evOriginal.y()-newHeight/2;
		ymax=evOriginal.y()+newHeight/2;
		if(xmin<0){
			xmax+=-xmin;
			xmin=0;
		}
		else if(xmax>internalImg->width()-1){
			xmin-=xmax-internalImg->width()+1;
			xmax=internalImg->width()-1;
		}
		if(ymin<0){
			ymax+=-ymin;
			ymin=0;
		}
		else if(ymax>internalImg->height()-1){
			ymin-=ymax-internalImg->height()+1;
			ymax=internalImg->height()-1;
		}
		mapMin=QPoint(xmin,ymin);
		mapMax=QPoint(xmax,ymax);
		
		ROS_ERROR("Update zoom - mapMin : %d %d",mapMin.x(),mapMin.y());
		ROS_ERROR("Update zoom - mapMax : %d %d",mapMax.x(),mapMax.y());
	}
	
	QPoint MapLoader::pointUnscaled(QPoint p){	//	Returns the point unscaled from UP-LEFT (in zoomed img)
		QPoint newPoint;
		float x=p.x();
		float y=p.y();
		float initialWidth=internalImg->width();
		float currentWidth=map->width();
		float climax=initialWidth/currentWidth;
		newPoint.setX(x*climax);
		newPoint.setY(y*climax);
		return newPoint;
	}
	
	void MapLoader::resetZoom(void){
		zoom=0;
		mapMin=QPoint(0,0);
		mapMax=QPoint(internalImg->width()-1,internalImg->height()-1);
	}
	
	QPoint MapLoader::getGlobalPoint(QPoint p){	//	Returns the point unscaled from DOWN-LEFT (in original img)
		QPoint np=pointUnscaled(p);
		int xev=np.x()/pow(2,zoom)+mapMin.x();
		int yev=np.y()/pow(2,zoom)+mapMin.y();
		return QPoint(xev,internalImg->height()-yev);
	}
	
	void MapLoader::wheelEvent ( QWheelEvent * event ){
		if(event->delta()>0){
			if(mapMax.y()+30<internalImg->height()){
				mapMax.setY(mapMax.y()+30);
				mapMin.setY(mapMin.y()+30);
			}
		}
		if(event->delta()<0){
			if(mapMin.y()-30>=0){
				mapMax.setY(mapMax.y()-30);
				mapMin.setY(mapMin.y()-30);
			}
		}
	}
}
