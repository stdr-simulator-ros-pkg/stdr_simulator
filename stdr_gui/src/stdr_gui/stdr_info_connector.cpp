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
        
    QObject::connect(
      this,
        SIGNAL(adaptSignal()),
      this,
        SLOT(adaptSlot()));
        
    QObject::connect(
      loader.stdrInformationTree, 
        SIGNAL(itemCollapsed(QTreeWidgetItem *)),
      this, 
        SLOT(adaptColumns(QTreeWidgetItem *)));
        
    QObject::connect(
      loader.stdrInformationTree, 
        SIGNAL(itemExpanded(QTreeWidgetItem *)),
      this, 
        SLOT(adaptColumns(QTreeWidgetItem *)));
  }

  void CInfoConnector::adaptColumns(QTreeWidgetItem *item, int column)
  {
    loader.stdrInformationTree->resizeColumnToContents(0);
    loader.stdrInformationTree->resizeColumnToContents(1);
    loader.stdrInformationTree->resizeColumnToContents(2);
    loader.stdrInformationTree->resizeColumnToContents(3);
  }
  
  void CInfoConnector::adaptColumns(QTreeWidgetItem *item)
  {
    loader.stdrInformationTree->resizeColumnToContents(0);
    loader.stdrInformationTree->resizeColumnToContents(1);
    loader.stdrInformationTree->resizeColumnToContents(2);
    loader.stdrInformationTree->resizeColumnToContents(3);
  }
  
  void CInfoConnector::adaptSlot(void)
  {
    loader.stdrInformationTree->resizeColumnToContents(0);
    loader.stdrInformationTree->resizeColumnToContents(1);
    loader.stdrInformationTree->resizeColumnToContents(2);
    loader.stdrInformationTree->resizeColumnToContents(3);
  }

  void CInfoConnector::updateMapInfo(float width,float height,float ocgd)
  {
    loader.updateMapInfo(width,height,ocgd);
    Q_EMIT adaptSignal();
  }
  
  void CInfoConnector::updateTree(
    const stdr_msgs::RobotIndexedVectorMsg& msg)
  {
    loader.deleteTree();
    loader.updateRobots(msg);
    Q_EMIT adaptSignal();
  }
  
  void CInfoConnector::treeItemClicked ( QTreeWidgetItem * item, int column )
  {
    adaptColumns(item, column);
    if(item == &loader.robotsInfo || item == &loader.generalInfo)
    {
      return;
    }
    else if(item->parent()->text(0) == QString("Lasers") && column == 3)
    {
      Q_EMIT laserVisualizerClicked(
        item->parent()->parent()->text(0), item->text(0));
    }
    else if(item->parent()->text(0) == QString("Lasers") && column == 2)
    {
      Q_EMIT laserVisibilityClicked(
        item->parent()->parent()->text(0), item->text(0));
    }
    else if(item->parent()->text(0) == QString("Sonars") && column == 3)
    {
      Q_EMIT sonarVisualizerClicked(
        item->parent()->parent()->text(0),item->text(0));
    }
    else if(item->parent()->text(0) == QString("Sonars") && column == 2)
    {
      Q_EMIT sonarVisibilityClicked(
        item->parent()->parent()->text(0),item->text(0));
    }
    else if(item->parent() == &loader.robotsInfo && column == 3)
    {
      Q_EMIT robotVisualizerClicked(
        item->text(0));
    }
    else if(item->parent() == &loader.robotsInfo && column == 2)
    {
      Q_EMIT robotVisibilityClicked(
        item->text(0));
    }
  }
  
  QWidget* CInfoConnector::getLoader(void)
  {
    return static_cast<QWidget *>(&loader);
  }
  
  void CInfoConnector::setLaserVisibility(
    QString robotName,QString laserName,char vs)
  {
    for(int i = 0 ; i < loader.robotsInfo.childCount() ; i++)
    {
      QString text = loader.robotsInfo.child(i)->text(0);
      if(text == robotName)
      {
        QTreeWidgetItem *it = loader.robotsInfo.child(i);
        for(int j = 0 ; j < it->childCount() ; j++)
        {
          if(it->child(j)->text(0) == QString("Lasers"))
          {
            for(int k = 0 ; k < it->child(j)->childCount() ; k++)
            {
              if(it->child(j)->child(k)->text(0) == laserName)
              {
                QTreeWidgetItem *inIt = it->child(j)->child(k);
                switch(vs)
                {
                  case 0:
                  {
                    inIt->setIcon(2,loader.visible_icon_on_);
                    break;
                  }
                  case 1:
                  {
                    inIt->setIcon(2,loader.visible_icon_trans_);
                    break;
                  }
                  case 2:
                  {
                    inIt->setIcon(2,loader.visible_icon_off_);
                    break;
                  }
                }
                return;
              }
            }
          }
        }
      }
    }
  }
  
  void CInfoConnector::setSonarVisibility(
    QString robotName,QString sonarName,char vs)
  {
    for(int i = 0 ; i < loader.robotsInfo.childCount() ; i++)
    {
      QString text = loader.robotsInfo.child(i)->text(0);
      if(text == robotName)
      {
        QTreeWidgetItem *it = loader.robotsInfo.child(i);
        for(int j = 0 ; j < it->childCount() ; j++)
        {
          if(it->child(j)->text(0) == QString("Sonars"))
          {
            for(int k = 0 ; k < it->child(j)->childCount() ; k++)
            {
              if(it->child(j)->child(k)->text(0) == sonarName)
              {
                QTreeWidgetItem *inIt = it->child(j)->child(k);
                switch(vs)
                {
                  case 0:
                  {
                    inIt->setIcon(2,loader.visible_icon_on_);
                    break;
                  }
                  case 1:
                  {
                    inIt->setIcon(2,loader.visible_icon_trans_);
                    break;
                  }
                  case 2:
                  {
                    inIt->setIcon(2,loader.visible_icon_off_);
                    break;
                  }
                }
                return;
              }
            }
          }
        }
      }
    }
  }  
  
  void CInfoConnector::setRobotVisibility(QString robotName,char vs)
  {
    for(int i = 0 ; i < loader.robotsInfo.childCount() ; i++)
    {
      QString text = loader.robotsInfo.child(i)->text(0);
      if(text == robotName)
      {
        switch(vs)
        {
          case 0:
          {
            loader.robotsInfo.child(i)->setIcon(2,loader.visible_icon_on_);
            break;
          }
          case 1:
          {
            loader.robotsInfo.child(i)->setIcon(2,loader.visible_icon_trans_);
            break;
          }
          case 2:
          {
            loader.robotsInfo.child(i)->setIcon(2,loader.visible_icon_off_);
            break;
          }
        }
        QTreeWidgetItem *it = loader.robotsInfo.child(i);
        for(int j = 0 ; j < it->childCount() ; j++)
        {
          if(it->child(j)->text(0) == QString("Sonars"))
          {
            for(int k = 0 ; k < it->child(j)->childCount() ; k++)
            {
              QTreeWidgetItem *inIt = it->child(j)->child(k);
              switch(vs)
              {
                case 0:
                {
                  inIt->setIcon(2,loader.visible_icon_on_);
                  break;
                }
                case 1:
                {
                  inIt->setIcon(2,loader.visible_icon_trans_);
                  break;
                }
                case 2:
                {
                  inIt->setIcon(2,loader.visible_icon_off_);
                  break;
                }
              }
            }
          }
          if(it->child(j)->text(0) == QString("Lasers"))
          {
            for(int k = 0 ; k < it->child(j)->childCount() ; k++)
            {
              QTreeWidgetItem *inIt = it->child(j)->child(k);
              switch(vs)
              {
                case 0:
                {
                  inIt->setIcon(2,loader.visible_icon_on_);
                  break;
                }
                case 1:
                {
                  inIt->setIcon(2,loader.visible_icon_trans_);
                  break;
                }
                case 2:
                {
                  inIt->setIcon(2,loader.visible_icon_off_);
                  break;
                }
              }
            }
          }
        }
        return;
      }
    }
  }
}
