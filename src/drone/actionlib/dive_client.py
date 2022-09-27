#!/usr/bin/env python3

import rospy
import actionlib
from drone.msg import DiveAction, DiveGoal


def action_feedback(fb):
    print(fb)


def dive_client():
    client = actionlib.SimpleActionClient('dive', DiveAction)
    client.wait_for_server()
    goal = DiveGoal(target=0)
    client.send_goal(goal, feedback_cb=action_feedback)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('dive_client')
        result = dive_client()
        print("Result: ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
