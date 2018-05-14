#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

from guiding_as.msg import *


def show_client():

    client = actionlib.SimpleActionClient('guiding_action_server', taskAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("wait for action server")
    client.wait_for_server()
    rospy.loginfo("send goal")
    # Creates a goal to send to the action server.
    goal = taskGoal(place_frame='door_A', person_frame="human-3")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('show_action_client')
    result = show_client()
    print(result)
