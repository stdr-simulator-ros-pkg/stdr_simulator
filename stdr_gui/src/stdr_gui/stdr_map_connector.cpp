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

namespace stdr{
	MapConnector::MapConnector(int argc, char **argv):
		QObject(),
		loader(argc,argv)
	{
		this->argc=argc;
		this->argv=argv;
		
		loader.map->setScaledContents(true);
		
		loader.map->installEventFilter(this);
		
		QObject::connect(this,SIGNAL(signalUpdateImage(QImage *)),this, SLOT(serveImage(QImage *)));
		
		QPixmap p((getRosPackagePath("stdr_gui")+std::string("/resources/images/zoom_in.png")).c_str());
		zoomInCursor=QCursor(p.scaled(20,20));
		p=QPixmap((getRosPackagePath("stdr_gui")+std::string("/resources/images/zoom_out.png")).c_str());
		zoomOutCursor=QCursor(p.scaled(20,20));
	}

	bool MapConnector::eventFilter( QObject* watched, QEvent* event ) {
		if(watched==loader.map){
			if(event->type() == QEvent::MouseButtonPress){
				const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
				if(me->button()==Qt::RightButton){
					//~ QMenu myMenu;
					//~ QAction *addR=myMenu.addAction("Add robot");
					//~ QAction *addRfid=myMenu.addAction("Add RFID tag");
					//~ QAction* selectedItem = myMenu.exec(loader.mapToGlobal(me->pos()));
				}
				else if(me->button()==Qt::LeftButton){
				}
			}
		}
		return false;
	}

	void MapConnector::updateImage(QImage *img){
		signalUpdateImage(img);
	}
	
	void MapConnector::serveImage(QImage *img){
		loader.updateImage(img);
	}
	
	void MapConnector::setCursorZoomIn(bool state){
		if(state)
			loader.map->setCursor(zoomInCursor);
		else
			loader.map->setCursor(QCursor(Qt::CrossCursor));
	}
	
	void MapConnector::setCursorZoomOut(bool state){
		if(state)
			loader.map->setCursor(zoomOutCursor);
		else
			loader.map->setCursor(QCursor(Qt::CrossCursor));
	}
}
