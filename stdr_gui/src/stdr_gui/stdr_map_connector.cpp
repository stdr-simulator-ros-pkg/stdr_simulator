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

#include "stdr_gui/stdr_map_connector.h"

namespace stdr_gui{
	MapConnector::MapConnector(int argc, char **argv):
		QObject(),
		_loader(argc,argv)
	{
		_argc=argc;
		_argv=argv;
		
		_mapState=NORMAL;
		
		_loader.map->setScaledContents(true);
		
		_loader.map->installEventFilter(this);
		
		QObject::connect(this,SIGNAL(signalUpdateImage(QImage *)),this, SLOT(serveImage(QImage *)));
		
		QPixmap p((getRosPackagePath("stdr_gui")+std::string("/resources/images/zoom_in.png")).c_str());
		_zoomInCursor=QCursor(p.scaled(20,20));
		p=QPixmap((getRosPackagePath("stdr_gui")+std::string("/resources/images/zoom_out.png")).c_str());
		_zoomOutCursor=QCursor(p.scaled(20,20));
	}
	
	void MapConnector::setupLoaderToGrid(QGridLayout *layout,int row,int column){
		layout->addWidget(static_cast<QWidget *>(&_loader),row,column,0);	
	}
	
	void MapConnector::setInitialImageSize(QSize s){
		_loader.initialImageSize=s;
	}
	
	void MapConnector::updateZoom(QPoint p,bool z){
		_loader.updateZoom(p,z);
	}
	
	QPoint MapConnector::getGlobalPoint(QPoint p){
		return _loader.getGlobalPoint(p);
	}
	
	void MapConnector::drawGrid(QImage *img,float resolution){
		_loader.drawGrid(img,resolution);
	}

	bool MapConnector::eventFilter( QObject* watched, QEvent* event ) {
		if(watched==_loader.map){
			if(event->type() == QEvent::MouseButtonPress){
				const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
				QPoint p=me->pos();
				if(me->button()==Qt::RightButton){
					if(_mapState==NORMAL){
						Q_EMIT itemClicked(p,Qt::RightButton);
					}
				}
				else if(me->button()==Qt::LeftButton){
					if(_mapState==ZOOMIN)
						Q_EMIT zoomInPressed(p);
					else if(_mapState==ZOOMOUT)
						Q_EMIT zoomOutPressed(p);
					else if(_mapState==SETPLACE){
						_mapState=NORMAL;
						_loader.map->setCursor(QCursor(Qt::CrossCursor));
						Q_EMIT robotPlaceSet(p);
					}
					else if(_mapState==NORMAL){
						Q_EMIT itemClicked(p,Qt::LeftButton);
					}
				}
			}
		}
		return false;
	}

	void MapConnector::updateImage(QImage *img){
		Q_EMIT signalUpdateImage(img);
	}
	
	void MapConnector::serveImage(QImage *img){
		_loader.updateImage(img);
	}
	
	QPoint MapConnector::mapToGlobal(QPoint p){
		return _loader.mapToGlobal(p);
	}
	
	void MapConnector::setCursorZoomIn(bool state){
		if(state){
			_mapState=ZOOMIN;
			_loader.map->setCursor(_zoomInCursor);
		}
		else{
			_mapState=NORMAL;
			_loader.map->setCursor(QCursor(Qt::CrossCursor));
		}
	}
	
	void MapConnector::setCursorZoomOut(bool state){
		if(state){
			_mapState=ZOOMOUT;
			_loader.map->setCursor(_zoomOutCursor);
		}
		else{
			_mapState=NORMAL;
			_loader.map->setCursor(QCursor(Qt::CrossCursor));
		}
	}
	
	void MapConnector::setCursorAdjusted(bool state){
		_loader.resetZoom();
		_mapState=NORMAL;
		_loader.map->setCursor(QCursor(Qt::CrossCursor));
	}
	
	void MapConnector::waitForPlace(void){
		_mapState=SETPLACE;
		_loader.map->setCursor(Qt::PointingHandCursor);
	}
}
