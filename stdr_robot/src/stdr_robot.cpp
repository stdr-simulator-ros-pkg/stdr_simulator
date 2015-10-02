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

    _odomPublisher = n.advertise<nav_msgs::Odometry>(getName() + "/odom", 10);

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

    //we should not start the timer, until we hame a motion controller
    _tfTimer = n.createTimer(
      ros::Duration(0.1), &Robot::publishTransforms, this, false, false);
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

    _currentPose = result->description.initialPose;

    _previousPose = _currentPose;

    for ( unsigned int laserIter = 0;
      laserIter < result->description.laserSensors.size(); laserIter++ )
    {
      _sensors.push_back( SensorPtr(
        new Laser( _map,
          result->description.laserSensors[laserIter], getName(), n ) ) );
    }
    for ( unsigned int sonarIter = 0;
      sonarIter < result->description.sonarSensors.size(); sonarIter++ )
    {
      _sensors.push_back( SensorPtr(
        new Sonar( _map,
          result->description.sonarSensors[sonarIter], getName(), n ) ) );
    }
    for ( unsigned int rfidReaderIter = 0;
      rfidReaderIter < result->description.rfidSensors.size(); 
        rfidReaderIter++ )
    {
      _sensors.push_back( SensorPtr(
        new RfidReader( _map,
          result->description.rfidSensors[rfidReaderIter], getName(), n ) ) );
    }
    for ( unsigned int co2SensorIter = 0;
      co2SensorIter < result->description.co2Sensors.size(); 
        co2SensorIter++ )
    {
      _sensors.push_back( SensorPtr(
        new CO2Sensor( _map,
          result->description.co2Sensors[co2SensorIter], getName(), n ) ) );
    }
    for ( unsigned int thermalSensorIter = 0;
      thermalSensorIter < result->description.thermalSensors.size(); 
        thermalSensorIter++ )
    {
      _sensors.push_back( SensorPtr(
        new ThermalSensor( _map,
          result->description.thermalSensors[thermalSensorIter], getName(), n ) ) );
    }
    for ( unsigned int soundSensorIter = 0;
      soundSensorIter < result->description.soundSensors.size(); 
        soundSensorIter++ )
    {
      _sensors.push_back( SensorPtr(
        new SoundSensor( _map,
          result->description.soundSensors[soundSensorIter], getName(), n ) ) );
    }

    if( result->description.footprint.points.size() == 0 ) {
      float radius = result->description.footprint.radius;
      for(unsigned int i = 0 ; i < 360 ; i++)
      {
        float x = cos(i * 3.14159265359 / 180.0) * radius;
        float y = sin(i * 3.14159265359 / 180.0) * radius;
        _footprint.push_back( std::pair<float,float>(x,y));
      }
    } else {
      for( unsigned int i = 0 ;
          i < result->description.footprint.points.size() ; 
          i++ ) {
        geometry_msgs::Point p = result->description.footprint.points[i];
        _footprint.push_back( std::pair<float,float>(p.x, p.y));
      }
    }

    std::string motion_model = result->description.kinematicModel.type;
    stdr_msgs::KinematicMsg p = result->description.kinematicModel;

    if(motion_model == "ideal")
    {
      _motionControllerPtr.reset(
        new IdealMotionController(_currentPose, _tfBroadcaster, n, getName(), p));
    }
    else if(motion_model == "omni")
    {
      _motionControllerPtr.reset(
        new OmniMotionController(_currentPose, _tfBroadcaster, n, getName(), p));
    }
    else
    {
      // If no motion model is specified or an invalid type declared use ideal
      _motionControllerPtr.reset(
        new IdealMotionController(_currentPose, _tfBroadcaster, n, getName(), p));
    }

    _tfTimer.start();
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
    if( collisionExistsNoPath(req.newPose) ||
        checkUnknownOccupancy(req.newPose) )
    {
      return false;
    }
    
    _currentPose = req.newPose;

    _previousPose = _currentPose;

    _motionControllerPtr->setPose(_previousPose);
    return true;
  }

  /**
  @brief Checks the robot collision -2b changed-
  @return True on collision
  **/
  bool Robot::collisionExistsNoPath(
    const geometry_msgs::Pose2D& newPose)
  {
    if(_map.info.width == 0 || _map.info.height == 0)
    {
      return false;
    }

    int xMap = newPose.x / _map.info.resolution;
    int yMap = newPose.y / _map.info.resolution;

    for(unsigned int i = 0 ; i < _footprint.size() ; i++)
    {
      double x = _footprint[i].first * cos(newPose.theta) -
                 _footprint[i].second * sin(newPose.theta);
      double y = _footprint[i].first * sin(newPose.theta) +
                 _footprint[i].second * cos(newPose.theta);
                 
      int xx = xMap + (int)(x / _map.info.resolution);
      int yy = yMap + (int)(y / _map.info.resolution);

      if(_map.data[ yy * _map.info.width + xx ] > 70)
      {
        return true;
      }
    }
    return false;
  }

  /**
  @brief Checks the robot's reposition into unknown area
  @param newPose [const geometry_msgs::Pose2D] The pose for the robot to be moved to
  @return True when position is in unknown area
  **/
  bool Robot::checkUnknownOccupancy(
    const geometry_msgs::Pose2D& newPose)
  {
    if(_map.info.width == 0 || _map.info.height == 0)
    {
      return false;
    }

    int xMap = newPose.x / _map.info.resolution;
    int yMap = newPose.y / _map.info.resolution;

    if( _map.data[ yMap * _map.info.width + xMap ] == -1 )
    {
      return true;
    }

    return false;
  }
  
  /**
  @brief Returns the points between two points
  @param x1 : The x coord of the first point
  @param y1 : The y coord of the first point
  @param x2 : The x coord of the second point
  @param y2 : The y coord of the second point
  @return The points inbetween
  **/
  std::vector<std::pair<int,int> > Robot::getPointsBetween(
    int x1, int y1, int x2, int y2) 
  {
    std::vector<std::pair<int,int> > points;
    
    float angle = atan2(y2 - y1, x2 - x1);
    float dist = sqrt( pow(x2 - x1, 2) + pow(y2 - y1, 2));
    
    int d = 0;

    while(d < dist)
    {
      int x = x1 + d * cos(angle);
      int y = y1 + d * sin(angle);
      points.push_back(std::pair<int,int>(x,y));
      d++;
    }
    
    return points;
  }

  /**
  @brief Checks the robot collision -2b changed-
  @return True on collision
  **/
  bool Robot::collisionExists(
    const geometry_msgs::Pose2D& newPose,
    const geometry_msgs::Pose2D& previousPose)
  {
    if(_map.info.width == 0 || _map.info.height == 0)
      return false;

    int xMapPrev, xMap, yMapPrev, yMap;
    bool movingForward, movingUpward;

    //Check robot's previous direction to prevent getting stuck when colliding
    movingForward = _previousMovementXAxis? false: true;
    if ( fabs(previousPose.x - newPose.x) > 0.001)
    {
      movingForward = (previousPose.x > newPose.x)? false: true;
      _previousMovementXAxis = movingForward;
    }
    movingUpward = _previousMovementYAxis? false: true;
    if ( fabs(previousPose.y - newPose.y) > 0.001)
    {
      movingUpward = (previousPose.y > newPose.y)? false: true;
      _previousMovementYAxis = movingUpward;
    }

    xMapPrev = movingForward? (int)( previousPose.x / _map.info.resolution ):
                              ceil( previousPose.x / _map.info.resolution );
    xMap = movingForward? ceil( newPose.x / _map.info.resolution ):
                          (int)( newPose.x / _map.info.resolution );

    yMapPrev = movingUpward? (int)( previousPose.y / _map.info.resolution ):
                              ceil( previousPose.y / _map.info.resolution );
    yMap = movingUpward? ceil( newPose.y / _map.info.resolution ):
                        (int)( newPose.y / _map.info.resolution );

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
      for(unsigned int i = 0 ; i < _footprint.size() ; i++)
      {
        int index_1 = i;
        int index_2 = (i + 1) % _footprint.size();
        
        // Get two consecutive footprint points
        double footprint_x_1 = _footprint[index_1].first * cos(newPose.theta) -
                   _footprint[index_1].second * sin(newPose.theta);
        double footprint_y_1 = _footprint[index_1].first * sin(newPose.theta) +
                   _footprint[index_1].second * cos(newPose.theta);

        int xx1 = x + footprint_x_1 / _map.info.resolution;
        int yy1 = y + footprint_y_1 / _map.info.resolution;
        
        double footprint_x_2 = _footprint[index_2].first * cos(newPose.theta) -
                   _footprint[index_2].second * sin(newPose.theta);
        double footprint_y_2 = _footprint[index_2].first * sin(newPose.theta) +
                   _footprint[index_2].second * cos(newPose.theta);

        int xx2 = x + footprint_x_2 / _map.info.resolution;
        int yy2 = y + footprint_y_2 / _map.info.resolution;
        
        //Here check all the points between the vertexes
        std::vector<std::pair<int,int> > pts = 
          getPointsBetween(xx1,yy1,xx2,yy2);
        
        for(unsigned int j = 0 ; j < pts.size() ; j++)
        {
          static int OF = 1;
          if(
            _map.data[ (pts[j].second - OF) * 
              _map.info.width + pts[j].first - OF ] > 70 ||
            _map.data[ (pts[j].second - OF) * 
              _map.info.width + pts[j].first ] > 70 ||
            _map.data[ (pts[j].second - OF) *  
              _map.info.width + pts[j].first + OF ] > 70 ||
            _map.data[ (pts[j].second) * 
              _map.info.width + pts[j].first - OF ] > 70 ||
            _map.data[ (pts[j].second) * 
              _map.info.width + pts[j].first + OF ] > 70 ||
            _map.data[ (pts[j].second + OF) * 
              _map.info.width + pts[j].first - OF ] > 70 ||
            _map.data[ (pts[j].second + OF) * 
              _map.info.width + pts[j].first ] > 70 ||
            _map.data[ (pts[j].second + OF) * 
              _map.info.width + pts[j].first + OF ] > 70
          )
          {
            return true;
          }
        }
      }
      if ( (movingForward && xMap < x) || (movingUpward && yMap < y) ||
          (!movingForward && xMap > x) || (!movingUpward && yMap > y) )
      {
        break;
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
    if( ! collisionExists(pose, _previousPose) )
    {
      _previousPose = pose;
    }
    else
    {
      _motionControllerPtr->setPose(_previousPose);
    }
    //!< Robot tf
    tf::Vector3 translation(_previousPose.x, _previousPose.y, 0);
    tf::Quaternion rotation;
    rotation.setRPY(0, 0, _previousPose.theta);

    tf::Transform mapToRobot(rotation, translation);

    _tfBroadcaster.sendTransform(tf::StampedTransform(
      mapToRobot, ros::Time::now(), "map_static", getName()));

    //!< Odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map_static";
    odom.child_frame_id = getName();
    odom.pose.pose.position.x = _previousPose.x;
    odom.pose.pose.position.y = _previousPose.y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        _previousPose.theta);
    odom.twist.twist = _motionControllerPtr->getVelocity();

    _odomPublisher.publish(odom);

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

}  // namespace stdr_robot
