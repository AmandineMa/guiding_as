#! /usr/bin/env python

import rospy

import actionlib

from move_base_msgs.msg import *
from mummer_integration_msgs.msg import *


class MoveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = MoveBaseActionFeedback()
    _result = MoveBaseActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        self._as.set_succeeded(self._result)


class Pointing(object):
    _feedback = PointingFeedback()
    _result = PointingResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PointingAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        self._result.pointed_landmarks = ['door_A', 'door']
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('action_server')
    # server = MoveAction('m_move_to')
    server = Pointing("pointing_planner/PointingPlanner")
    rospy.spin()
