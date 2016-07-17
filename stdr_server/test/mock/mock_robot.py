#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer, SimpleActionClient

from nodelet.srv import NodeletLoad, NodeletUnload
from stdr_msgs.msg import SpawnRobotAction, SpawnRobotResult
from stdr_msgs.msg import RegisterRobotAction, RegisterRobotGoal
from stdr_msgs.srv import MoveRobot, MoveRobotResponse

import threading

class MockRobot():

    def __init__(self):
        self.load_nodelet = rospy.Service('robot_manager/load_nodelet',
                NodeletLoad, self.load_nodelet_cb)

        self.unload_nodelet = rospy.Service('robot_manager/unload_nodelet',
                NodeletUnload, self.unload_nodelet_cb)

        self.register_robot_client = \
            SimpleActionClient('stdr_server/register_robot', RegisterRobotAction)
        self.register_robot_client.wait_for_server()

        self.name = None

    def load_nodelet_cb(self, request):
        rospy.loginfo('Requested new robot')
        # Spawn a new thread, as this service has to return first
        req_thread = threading.Thread(target=self.register_robot, args=(request.name,))
        req_thread.start()
        return True

    def unload_nodelet_cb(self, request):
        rospy.loginfo('Request to delete "%s"' % request.name)
        self.move_service.shutdown()
        return True

    def register_robot(self, name):
        rospy.sleep(1)
        rospy.loginfo('Register "%s" to server' % name)
        register_goal = RegisterRobotGoal()
        register_goal.name = name
        self.register_robot_client.send_goal(register_goal)
        self.name = name
        # Spawn move robot service
        self.move_service = rospy.Service(name + '/replace', MoveRobot,
                                self.move_robot)

    def move_robot(self, req):
        rospy.loginfo('Request to move %s' % self.name)
        return MoveRobotResponse()

if __name__ == '__main__':
    rospy.init_node('mock_robot', anonymous=True)
    robot = MockRobot()
    rospy.spin()
