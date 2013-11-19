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

#ifndef MOTION_CONTROLLER_BASE_H
#define MOTION_CONTROLLER_BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

namespace stdr_robot {
	
class MotionController {
	
	public:
		
		virtual void velocityCallback(const geometry_msgs::Twist& msg) = 0;
		virtual void stop() = 0; 
	
	protected:
		
		MotionController(geometry_msgs::Pose2DPtr pose, tf::TransformBroadcaster& tf)
		: _posePtr(pose), _tfBroadcaster(tf) {}
	
	protected:
	
		tf::TransformBroadcaster& _tfBroadcaster;
		geometry_msgs::Pose2DPtr _posePtr;

};
	
typedef boost::shared_ptr<MotionController> MotionControllerPtr;
	
}


#endif
