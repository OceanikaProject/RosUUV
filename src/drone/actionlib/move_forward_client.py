#!/usr/bin/env python3

import rospy
import actionlib
from drone.msg import MoveForwardAction, MoveForwardGoal


def action_feedback(fb):
    print(fb)


def move_forward_client():
    client = actionlib.SimpleActionClient('move_forward', MoveForwardAction)
    client.wait_for_server()
    goal = MoveForwardGoal(seconds=3)
    client.send_goal(goal, feedback_cb=action_feedback)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('move_forward_client')
        result = move_forward_client()
        print("Result: ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
