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

#include <stdr_server/stdr_server.h>

namespace stdr_server {
	
Server::Server(int argc, char** argv) 
{
	
	if (argc > 2) {
		ROS_ERROR("%s", USAGE);
		exit(-1);
	}
	
	if (argc == 2) {
		std::string fname(argv[1]);
		_mapServer.reset(new stdr_server::MapServer(fname));
	}
		
	_loadMapService = _nh.advertiseService("/stdr_server/load_static_map", &Server::loadMapCallback, this);
	
}

bool Server::loadMapCallback(stdr_msgs::LoadMap::Request& req,
							stdr_msgs::LoadMap::Response& res) 
{
	if (_mapServer) {
		ROS_WARN("Map already loaded!");
		return false;
	}
	_mapServer.reset(new stdr_server::MapServer(req.mapFile));

	return true;	
}
	
}
