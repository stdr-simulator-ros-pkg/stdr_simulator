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

#include <stdr_robot/stdr_robot.h>
#include <nodelet/NodeletUnload.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(stdr_robot::Robot, nodelet::Nodelet)

namespace stdr_robot
{
  /**
  @brief Default constructor
  @return void
  **/
  Robot::Robot(void)
  {

  }

  /**
  @brief Initializes the robot and gets the environment occupancy grid map
  @return void
  **/
  void Robot::onInit()
  {
    ros::NodeHandle n = getMTNodeHandle();
    _currentPosePtr.reset( new geometry_msgs::Pose2D );

    _registerClientPtr.reset(
      new RegisterRobotClient(n, "stdr_server/register_robot", true) );

    _registerClientPtr->waitForServer();

    stdr_msgs::RegisterRobotGoal goal;
    goal.name = getName();
    _registerClientPtr->sendGoal(goal,
      boost::bind(&Robot::initializeRobot, this, _1, _2));	

    _mapSubscriber = n.subscribe("map", 1, &Robot::mapCallback, this);
    _moveRobotService = n.advertiseService(
      getName() + "/replace", &Robot::moveRobotCallback, this);

    _tfTimer = n.createTimer(
      ros::Duration(0.1), &Robot::publishTransforms, this);
  }

  /**
  @brief Initializes the robot after on registering it to server
  @param state [const actionlib::SimpleClientGoalState&] State of action
  @param result [const stdr_msgs::RegisterRobotResultConstPtr] Action result of registering the robot
  @return void
  **/
  void Robot::initializeRobot(
    const actionlib::SimpleClientGoalState& state,
    const stdr_msgs::RegisterRobotResultConstPtr result)
  {

    if (state == state.ABORTED) {
      NODELET_ERROR("Something really bad happened...");
      return;
    }

    NODELET_INFO("Loaded new robot, %s", getName().c_str());
    ros::NodeHandle n = getMTNodeHandle();

    _currentPosePtr->x = result->description.initialPose.x;
    _currentPosePtr->y = result->description.initialPose.y;
    _currentPosePtr->theta = result->description.initialPose.theta;

    previous_pose.x = _currentPosePtr->x;
    previous_pose.y = _currentPosePtr->y;
    previous_pose.theta = _currentPosePtr->theta;

    for ( unsigned int laserIter = 0;
      laserIter < result->description.laserSensors.size(); laserIter++ ){
      _sensors.push_back( SensorPtr(
        new Laser( _map,
          result->description.laserSensors[laserIter], getName(), n ) ) );
    }
    for ( unsigned int sonarIter = 0;
      sonarIter < result->description.sonarSensors.size(); sonarIter++ ){
      _sensors.push_back( SensorPtr(
        new Sonar( _map,
          result->description.sonarSensors[sonarIter], getName(), n ) ) );
    }

    float radius = result->description.footprint.radius;
    for(unsigned int i = 0 ; i < 360 ; i++)
    {
      float x = cos(i * 3.14159265359 / 180.0) * radius;
      float y = sin(i * 3.14159265359 / 180.0) * radius;
      footprint.push_back( std::pair<float,float>(x,y));
    }

    _motionControllerPtr.reset(
      new IdealMotionController(_currentPosePtr, _tfBroadcaster, n, getName()));
  }

  /**
  @brief Callback for getting the occupancy grid map
  @param msg [const nav_msgs::OccupancyGridConstPtr&] The occupancy grid map
  @return void
  **/
  void Robot::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    _map = *msg;
  }

  /**
  @brief The callback of the re-place robot service
  @param req [stdr_msgs::MoveRobot::Request&] The service request
  @param res [stdr_msgs::MoveRobot::Response&] The service result
  @return bool
  **/
  bool Robot::moveRobotCallback(stdr_msgs::MoveRobot::Request& req,
                stdr_msgs::MoveRobot::Response& res)
  {

    if(collisionExistsNoPath(req.newPose))
    {
      return false;
    }

    _currentPosePtr->x = req.newPose.x;
    _currentPosePtr->y = req.newPose.y;

    previous_pose.x = _currentPosePtr->x;
    previous_pose.y = _currentPosePtr->y;

    _motionControllerPtr->setPose(previous_pose);
    return true;
  }

  /**
  @brief Checks the robot collision -2b changed-
  @return True on collision
  **/
  bool Robot::collisionExistsNoPath(
    geometry_msgs::Pose2D new_pose)
  {
    if(_map.info.width == 0 || _map.info.height == 0)
    {
      return false;
    }

    int xMap = new_pose.x / _map.info.resolution;
    int yMap = new_pose.y / _map.info.resolution;

    int x = xMap;
    int y = yMap;

    for(unsigned int i = 0 ; i < footprint.size() ; i++)
    {
      int xx = x + (int)(footprint[i].first / _map.info.resolution);
      int yy = y + (int)(footprint[i].second / _map.info.resolution);

      if(_map.data[ yy * _map.info.width + xx ] > 70)
      {
        return true;
      }
    }
    return false;
  }

  /**
  @brief Checks the robot collision -2b changed-
  @return True on collision
  **/
  bool Robot::collisionExists(
    geometry_msgs::Pose2D new_pose,
    geometry_msgs::Pose2D& previous_pose)
  {
    if(_map.info.width == 0 || _map.info.height == 0)
    {
      return false;
    }

    int xMapPrev, xMap, yMapPrev, yMap;
    bool movingForward = false, movingUpward = false;
    if ( previous_pose.x < new_pose.x )
      movingForward = true;
    if ( previous_pose.y < new_pose.y )
      movingUpward = true;

    xMapPrev = movingForward? (int)( previous_pose.x / _map.info.resolution ):
                              ceil( previous_pose.x / _map.info.resolution );
    xMap = movingForward? ceil( new_pose.x / _map.info.resolution ):
                          (int)( new_pose.x / _map.info.resolution );

    yMapPrev = movingUpward? (int)( previous_pose.y / _map.info.resolution ):
                              ceil( previous_pose.y / _map.info.resolution );
    yMap = movingUpward? ceil( new_pose.y / _map.info.resolution ):
                        (int)( new_pose.y / _map.info.resolution );

    float angle = atan2(yMap - yMapPrev, xMap - xMapPrev);
    int x = xMapPrev;
    int y = yMapPrev;
    int d = 2;

    while(pow(xMap - x,2) + pow(yMap - y,2) > 1)
    {
      x = xMapPrev +
        ( movingForward? ceil( cos(angle) * d ): (int)( cos(angle) * d ) );
      y = yMapPrev +
        ( movingUpward? ceil( sin(angle) * d ): (int)( sin(angle) * d ) );
      //Check all footprint points
      for(unsigned int i = 0 ; i < footprint.size() ; i++)
      {
        int xx = x + footprint[i].first / _map.info.resolution;
        int yy = y + footprint[i].second / _map.info.resolution;

        if(_map.data[ yy * _map.info.width + xx ] > 70)
        {
          return true;
        }
      }
      d++;
    }
    return false;
  }

  /**
  @brief Publishes the tf transforms every with 10Hz
  @return void
  **/
  void Robot::publishTransforms(const ros::TimerEvent&)
  {
    geometry_msgs::Pose2D pose = _motionControllerPtr->getPose();
    if( ! collisionExists(pose, previous_pose) )
    {
      previous_pose = pose;
    }
    else
    {
      _motionControllerPtr->setPose(previous_pose);
    }
    //!< Robot tf
    tf::Vector3 translation(previous_pose.x, previous_pose.y, 0);
    tf::Quaternion rotation;
    rotation.setRPY(0, 0, previous_pose.theta);

    tf::Transform mapToRobot(rotation, translation);

    _tfBroadcaster.sendTransform(tf::StampedTransform(
      mapToRobot, ros::Time::now(), "map_static", getName()));

    //!< Sensors tf
    for (int i = 0; i < _sensors.size(); i++) {
      geometry_msgs::Pose2D sensorPose = _sensors[i]->getSensorPose();

      tf::Vector3 trans(sensorPose.x, sensorPose.y, 0);
      tf::Quaternion rot;
      rot.setRPY(0, 0, sensorPose.theta);

      tf::Transform robotToSensor(rot, trans);

      _tfBroadcaster.sendTransform(
        tf::StampedTransform(
          robotToSensor,
          ros::Time::now(),
          getName(),
          _sensors[i]->getFrameId()));
    }
  }

  /**
  @brief Default destructor
  @return void
  **/
  Robot::~Robot()
  {
    //!< Cleanup
  }

}
