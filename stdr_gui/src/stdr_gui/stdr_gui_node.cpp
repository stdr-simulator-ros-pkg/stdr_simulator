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

#include <signal.h>
#include "stdr_gui/stdr_gui_controller.h"
#include "stdr_gui/stdr_gui_application.h"

void signalHandler(int sig);

/**
@brief The main node function
@param argc [int] Number of input arguments
@param argv [char] The input arguments
@return int : 0 for success
**/
int main(int argc,char **argv)
{
  stdr_gui::CStdrApplication app(argc, argv);
  app.setAttribute(Qt::AA_DontShowIconsInMenus, false);
  ros::init(argc, argv, "stdr_gui_node", ros::init_options::NoSigintHandler);
  stdr_gui::CGuiController con(argc, argv);

  // Add custom signal handlers
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);
  
  con.init();
  app.exec();
  return 0;
}

/**
@brief Signal handler, kills QApplication
**/
void signalHandler(int sig)
{
  QApplication::quit();
}
