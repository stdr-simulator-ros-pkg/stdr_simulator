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
		loader.map->installEventFilter( this );
	}
	
	bool MapConnector::eventFilter( QObject* watched, QEvent* event ) {
		
		if(watched==loader.map){
			if(event->type() == QEvent::MouseButtonPress){

				const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
				//~ std::vector<float> r=fromMapPoint(me->pos());
				
				if(me->button()==Qt::RightButton){

					//~ QPoint globalPos = loader.myWidget->mapToGlobal(me->pos());
					//~ globalPos.setX(globalPos.x()+400);
					//~ 
					//~ if(checkProximityWithObject(me->pos())){
						//~ globalPause=false;
						//~ return false;
					//~ }
					
					QMenu myMenu;

					QAction *addR=myMenu.addAction("Add robot");
					QAction *addRfid=myMenu.addAction("Add RFID tag");

					QAction* selectedItem = myMenu.exec(loader.mapToGlobal(me->pos()));
					//~ if (selectedItem==addR){
						//~ addRobot(r[0],r[1]);
					//~ }
					//~ else if(selectedItem==addRfid){
						//~ rfidTag tag;
						//~ tag.x=r[0];
						//~ tag.y=r[1];
						//~ int iid=-1;
						//~ for(unsigned int k=0;k<tags.size();k++)
							//~ if(tags[k].id>=iid) iid=tags[k].id;
						//~ tag.id=iid+1;
						//~ tags.push_back(tag);
					//~ }
					//~ else
					//~ {
						//~ // nothing was chosen
					//~ }
					//~ usleep(100000);
					//~ globalPause=false;
				}
				else if(me->button()==Qt::LeftButton){
					//~ int x=r[0];
					//~ int y=r[1];
					//~ for(unsigned int r=0;r<robots.size();r++){
						//~ if(cPixelCoords(robots[r].x,robots[r].y).distanceFrom(cPixelCoords(x,y))<15)
							//~ robots[r].enableInfo=!robots[r].enableInfo;
					//~ }
					//~ for(unsigned int r=0;r<tags.size();r++){
						//~ if(cPixelCoords(tags[r].x,tags[r].y).distanceFrom(cPixelCoords(x,y))<15)
							//~ tags[r].enableInfo=!tags[r].enableInfo;
					//~ }
				}

			}
		}

			
		
		return false;
	}
}
