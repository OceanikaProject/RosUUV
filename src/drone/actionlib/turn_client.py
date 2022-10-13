#!/usr/bin/env python3

import rospy
import actionlib
from drone.msg import TurnAction, TurnGoal


def action_feedback(fb):
    print(fb)


def turn_client():
    client = actionlib.SimpleActionClient('turn', TurnAction)
    client.wait_for_server()
    goal = TurnGoal(target='left')
    client.send_goal(goal, feedback_cb=action_feedback)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('turn_client')
        result = turn_client()
        print("Result: ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
