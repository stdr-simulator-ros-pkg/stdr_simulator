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

#include "stdr_gui/stdr_info_connector.h"

namespace stdr_gui
{

  CInfoConnector::CInfoConnector(int argc, char **argv):
    QObject(),
    loader(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    QObject::connect(
      loader.stdrInformationTree,
        SIGNAL(itemClicked(QTreeWidgetItem*, int)),
      this,
        SLOT(treeItemClicked(QTreeWidgetItem*, int)));
  }

  void CInfoConnector::updateMapInfo(float width,float height,float ocgd)
  {
    loader.updateMapInfo(width,height,ocgd);
  }
  
  void CInfoConnector::updateTree(
    const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
    loader.deleteTree();
    loader.updateRobots(msg);
  }
  
  void CInfoConnector::treeItemClicked ( QTreeWidgetItem * item, int column )
  {
    if(item == &loader.robotsInfo)
    {
      return;
    }
    else if(item->parent()->text(0) == QString("Lasers") && column == 3)
    {
      Q_EMIT laserVisualizerClicked(
        item->parent()->parent()->text(0), item->text(0));
    }
    else if(item->parent()->text(0) == QString("Sonars") && column == 3)
    {
      Q_EMIT sonarVisualizerClicked(
        item->parent()->parent()->text(0),item->text(0));
    }
    else if(item->parent() == &loader.robotsInfo)
    {
      Q_EMIT robotVisualizerClicked(item->text(0));
    }
  }
  
  QWidget* CInfoConnector::getLoader(void)
  {
    return static_cast<QWidget *>(&loader);
  }
}
