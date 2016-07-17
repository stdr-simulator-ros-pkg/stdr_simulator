#!/usr/bin/env python

import sys
import rospy
import rostest
import unittest
from actionlib import SimpleActionClient
import actionlib

from stdr_msgs.msg import RobotMsg, RobotIndexedVectorMsg
from stdr_msgs.msg import (SpawnRobotAction, SpawnRobotGoal,
                            DeleteRobotAction, DeleteRobotGoal)
from stdr_msgs.srv import LoadMap, MoveRobot, MoveRobotRequest
from stdr_msgs.srv import (AddCO2Source, AddRfidTag, AddSoundSource,
                            AddThermalSource, DeleteCO2Source, DeleteRfidTag,
                            DeleteSoundSource, DeleteThermalSource)
from stdr_msgs.msg import (CO2Source, CO2SourceVector, ThermalSource,
                            ThermalSourceVector)
from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose2D

from subprocess import call
from collections import defaultdict
import threading

class InterfaceTest(unittest.TestCase):

    @classmethod
    def setUp(cls):
        print 'Starting...'
        cls.messages = defaultdict(list)

    @classmethod
    def mock_cb(cls, msg, topic):
        rospy.loginfo('got message from %s' % topic)
        cls.messages[topic].append(msg)
        cls.block.set()

    @classmethod
    def load_map(cls, map_filename):
        command = 'rosrun stdr_server load_map %s' % map_filename
        call(command, shell=True)

    @classmethod
    def connect(cls):
        cls.spawn_robot_client = \
            SimpleActionClient('stdr_server/spawn_robot',
                                            SpawnRobotAction)
        cls.spawn_robot_client.wait_for_server(rospy.Duration(5.0))

        cls.delete_robot_client = \
            SimpleActionClient('stdr_server/delete_robot', DeleteRobotAction)
        cls.delete_robot_client.wait_for_server(rospy.Duration(5.0))

        cls.robots_topic = 'stdr_server/active_robots'
        cls.active_robots_sub = rospy.Subscriber(cls.robots_topic,
                    RobotIndexedVectorMsg, cls.mock_cb, cls.robots_topic)

        # sources
        cls.add_rfid = rospy.ServiceProxy('stdr_server/add_rfid_tag',
                                            AddRfidTag)
        cls.delete_rfid = rospy.ServiceProxy('stdr_server/delete_rfid_tag',
                                                DeleteRfidTag)
        cls.add_co2 = rospy.ServiceProxy('stdr_server/add_co2_source',
                                            AddCO2Source)
        cls.delete_co2 = rospy.ServiceProxy('stdr_server/delete_co2_source',
                                            DeleteCO2Source)
        cls.add_thermal = rospy.ServiceProxy('stdr_server/add_thermal_source',
                                                AddThermalSource)
        cls.delete_thermal = rospy.ServiceProxy('stdr_server/delete_thermal_source',
                                            DeleteThermalSource)
        cls.add_sound = rospy.ServiceProxy('stdr_server/add_sound_source',
                                            AddSoundSource)
        cls.delete_sound = rospy.ServiceProxy('stdr_server/delete_sound_source',
                                                DeleteSoundSource)

        cls.block = threading.Event()

    def _add_robot(self):
        self.block.clear()
        spawn_robot_goal = SpawnRobotGoal()
        robot_msg = RobotMsg()
        robot_msg.footprint.radius = 0.2
        robot_msg.initialPose.x = 1
        robot_msg.initialPose.y = 2
        spawn_robot_goal.description = robot_msg

        self.spawn_robot_client.send_goal(spawn_robot_goal)

        self.assertTrue(self.spawn_robot_client.wait_for_result(rospy.Duration(10.0)))
        self.assertFalse(
                self.spawn_robot_client.get_state() == actionlib.GoalStatus.ABORTED)

        robot_name = \
            self.spawn_robot_client.get_result().indexedDescription.name

        # wait until we get a message
        self.block.wait()

        self.assertIn(self.robots_topic, self.messages.keys())
        self.assertEqual(len(self.messages[self.robots_topic]), 1)
        self.assertEqual(len(self.messages[self.robots_topic][-1].robots), 1)
        self.assertEqual(self.messages[self.robots_topic][-1].robots[0].name, robot_name)
        self.assertAlmostEqual(
                self.messages[self.robots_topic][-1].robots[0].robot.footprint.radius,
                                robot_msg.footprint.radius)
        self.assertAlmostEqual(
                self.messages[self.robots_topic][-1].robots[0].robot.initialPose.x,
                                robot_msg.initialPose.x)
        self.assertAlmostEqual(
                self.messages[self.robots_topic][-1].robots[0].robot.initialPose.y,
                                robot_msg.initialPose.y)

        return robot_name

    def _delete_robot(self, name):
        self.block.clear()
        delete_robot_goal = DeleteRobotGoal()
        delete_robot_goal.name = name
        self.delete_robot_client.send_goal(delete_robot_goal)
        self.assertTrue(self.delete_robot_client.wait_for_result(rospy.Duration(10.0)))

        # wait until we get a message
        self.block.wait()
        self.assertEqual(
                len(self.messages[self.robots_topic][-1].robots), 0)

    def test_map_loaded(self):
        self.block.clear()
        self.map_topic = 'map'
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.mock_cb,
                self.map_topic)

        # wait until we get a message
        self.block.wait()
        self.assertIn(self.map_topic, self.messages.keys())
        self.assertEqual(len(self.messages[self.map_topic]), 1)

        # cleanup
        self.map_sub.unregister()

    def test_add_delete_robot(self):
        name = self._add_robot()
        self._delete_robot(name)

    def test_move_robot(self):
        # first we need to add a new robot
        name = self._add_robot()

        rospy.wait_for_service(name + '/replace')
        move_service = rospy.ServiceProxy(name + '/replace', MoveRobot)
        pose = Pose2D(x=2, y=4)
        req = MoveRobotRequest(newPose=pose)
        self.assertTrue(move_service(req))

        # cleanup by deleting the robot
        self._delete_robot(name)

    def test_add_delete_sources(self):
        self.block.clear()

        # markers
        self.markers_topic = 'stdr_server/sources_visualization_markers'
        self.markers_sub = rospy.Subscriber(self.markers_topic, MarkerArray,
                                            self.mock_cb, self.markers_topic)
        self.co2_list_topic = 'stdr_server/co2_sources_list'
        self.co2_sub = rospy.Subscriber(self.co2_list_topic, CO2SourceVector,
                                        self.mock_cb, self.co2_list_topic)

        self.thermal_list_topic = 'stdr_server/thermal_sources_list'
        self.thermal_sub = rospy.Subscriber(self.thermal_list_topic,
                                            ThermalSourceVector,
                                            self.mock_cb, self.thermal_list_topic)

        # wait for connections to be established
        rospy.sleep(2.0)

        ### test co2 addition
        new_co2_source = CO2Source(id='1', ppm=1000, pose=Pose2D(x=1, y=1))
        self.add_co2(newSource=new_co2_source)

        rospy.sleep(2.0) # wait for multiple msgs

        self.assertIn(self.co2_list_topic, self.messages.keys())
        self.assertEqual(len(self.messages[self.co2_list_topic]), 1)
        # expect 2 identical messages due to sources republication
        self.assertEqual(len(self.messages[self.markers_topic]), 2)

        co2_msg = self.messages[self.co2_list_topic][-1]
        self.assertEqual(co2_msg.co2_sources[0].id, '1')
        self.assertEqual(co2_msg.co2_sources[0].pose, Pose2D(x=1, y=1))

        self.assertIn(self.markers_topic, self.messages.keys())
        co2_marker_msg = self.messages[self.markers_topic][-1].markers[-1]
        self.assertEqual(co2_marker_msg.ns, 'co2')
        self.assertEqual(co2_marker_msg.type, 2)


        ### test thermal addition
        new_thermal_source = \
            ThermalSource(id='1', degrees=180, pose=Pose2D(x=2, y=2))
        self.add_thermal(newSource=new_thermal_source)

        rospy.sleep(2.0)

        self.assertIn(self.thermal_list_topic, self.messages.keys())
        thermal_msg = self.messages[self.thermal_list_topic][-1]
        self.assertEqual(thermal_msg.thermal_sources[0].id, '1')
        self.assertEqual(thermal_msg.thermal_sources[0].pose, Pose2D(x=2, y=2))

        thermal_marker_msg = self.messages[self.markers_topic][-1].markers[-1]
        self.assertEqual(co2_marker_msg.ns, 'co2')
        self.assertTrue(any(
            (True for mrk in self.messages[self.markers_topic][-1].markers
                if mrk.ns == 'thermal')))


        ### delete the thermal source, we should still get a marker for co2
        self.delete_thermal(name='1')

        rospy.sleep(2.0)

        thermal_msg = self.messages[self.thermal_list_topic][-1]
        co2_msg = self.messages[self.co2_list_topic][-1]
        self.assertFalse(thermal_msg.thermal_sources)
        self.assertTrue(co2_msg.co2_sources)

        # last marker msg should contain one co2 source
        marker_msg = self.messages[self.markers_topic][-1].markers
        self.assertEqual(marker_msg[-1].ns, 'co2')


        ### test invalid source id addition
        new_co2_source = CO2Source(id='1', ppm=1000, pose=Pose2D(x=3, y=3))
        self.assertRaises(rospy.ServiceException, self.add_co2, new_co2_source)

        # we should get no changes in co2 list msg
        co2_msg = self.messages[self.co2_list_topic][-1]
        self.assertEqual(len(co2_msg.co2_sources), 1)
        self.assertEqual(co2_msg.co2_sources[-1].id, '1')
        self.assertEqual(co2_msg.co2_sources[-1].pose, Pose2D(x=1, y=1))


if __name__ == '__main__':
    rospy.init_node('interface_test', anonymous=True, log_level=rospy.INFO)
    InterfaceTest.connect()
    rospy.sleep(2.0)
    rostest.rosrun('stdr_server', 'interface_test', InterfaceTest, sys.argv)
