#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

import guiding_as.msg


def show_client():

    client = actionlib.SimpleActionClient('guiding_action_server', guiding_as.msg.taskAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("wait for action server")
    client.wait_for_server()
    rospy.loginfo("send goal")
    # Creates a goal to send to the action server.
    goal = guiding_as.msg.taskGoal(place_frame='stairs_1', person_frame="human-8")

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
