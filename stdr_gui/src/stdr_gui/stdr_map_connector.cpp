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
	
	CMapConnector::CMapConnector(int argc, char **argv):
		QObject(),
		loader_(argc,argv),
		argc_(argc),
		argv_(argv)
	{
		map_state_=NORMAL;
		
		loader_.map->setScaledContents(true);
		
		loader_.map->installEventFilter(this);
		
		QObject::connect(
			this,SIGNAL(signalUpdateImage(QImage *)),
			this, SLOT(serveImage(QImage *)));
		
		QPixmap p((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
			std::string("/resources/images/zoom_in.png")).c_str());
		zoom_in_cursor_=QCursor(p.scaled(20,20));
		
		p=QPixmap((
			stdr_gui_tools::getRosPackagePath("stdr_gui")+
			std::string("/resources/images/zoom_out.png")).c_str());
		zoom_out_cursor_=QCursor(p.scaled(20,20));
	}
	
	CMapConnector::~CMapConnector(void)
	{
		
	}
	
	void CMapConnector::setInitialImageSize(QSize s)
	{
		loader_.setInitialImageSize(s);
	}
	
	void CMapConnector::updateZoom(QPoint p,bool z)
	{
		loader_.updateZoom(p,z);
	}
	
	void CMapConnector::updateCenter(QPoint p){
		loader_.updateCenter(p);
	}
	
	QPoint CMapConnector::getGlobalPoint(QPoint p)
	{
		return loader_.getGlobalPoint(p);
	}
	
	void CMapConnector::drawGrid(QImage *img,float resolution)
	{
		loader_.drawGrid(img,resolution);
	}

	bool CMapConnector::eventFilter( QObject* watched, QEvent* event ) 
	{
		if(watched==loader_.map)
		{
			if(event->type() == QEvent::MouseButtonPress)
			{
				const QMouseEvent* const me = 
					static_cast<const QMouseEvent*>( event );
				QPoint p=me->pos();
				if(me->button()==Qt::RightButton)
				{
					if(map_state_==NORMAL)
					{
						Q_EMIT itemClicked(p,Qt::RightButton);
					}
				}
				else if(me->button()==Qt::LeftButton)
				{
					if(map_state_==ZOOMIN)
					{
						Q_EMIT zoomInPressed(p);
					}
					else if(map_state_==ZOOMOUT)
					{
						Q_EMIT zoomOutPressed(p);
					}
					else if(map_state_==SETPLACE)
					{
						map_state_=NORMAL;
						loader_.map->setCursor(QCursor(Qt::CrossCursor));
						Q_EMIT robotPlaceSet(p);
					}
					else if(map_state_==NORMAL)
					{
						Q_EMIT itemClicked(p,Qt::LeftButton);
					}
					else if(map_state_==SETREPLACE)
					{
						map_state_=NORMAL;
						loader_.map->setCursor(QCursor(Qt::CrossCursor));
						Q_EMIT robotReplaceSet(p,current_robot_frame_id_);
					}
				}
			}
		}
		return false;
	}

	void CMapConnector::updateImage(QImage *img)
	{
		Q_EMIT signalUpdateImage(img);
	}
	
	void CMapConnector::serveImage(QImage *img)
	{
		loader_.updateImage(img);
	}
	
	QPoint CMapConnector::mapToGlobal(QPoint p)
	{
		return loader_.mapToGlobal(p);
	}
	
	void CMapConnector::setCursorZoomIn(bool state)
	{
		if(state)
		{
			map_state_=ZOOMIN;
			loader_.map->setCursor(zoom_in_cursor_);
		}
		else
		{
			map_state_=NORMAL;
			loader_.map->setCursor(QCursor(Qt::CrossCursor));
		}
	}
	
	void CMapConnector::setCursorZoomOut(bool state)
	{
		if(state)
		{
			map_state_=ZOOMOUT;
			loader_.map->setCursor(zoom_out_cursor_);
		}
		else
		{
			map_state_=NORMAL;
			loader_.map->setCursor(QCursor(Qt::CrossCursor));
		}
	}
	
	void CMapConnector::setCursorAdjusted(bool state)
	{
		loader_.resetZoom();
		map_state_=NORMAL;
		loader_.map->setCursor(QCursor(Qt::CrossCursor));
	}
	
	void CMapConnector::waitForPlace(void)
	{
		map_state_=SETPLACE;
		loader_.map->setCursor(Qt::PointingHandCursor);
	}
	
	QWidget* CMapConnector::getLoader(void)
	{
		return static_cast<QWidget *>(&loader_);
	}
	
	void CMapConnector::waitForReplace(std::string robotFrameId){
		current_robot_frame_id_=robotFrameId;
		map_state_=SETREPLACE;
		loader_.map->setCursor(Qt::PointingHandCursor);
	}
}
