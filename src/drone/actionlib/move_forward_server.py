#!/usr/bin/env python3

import rospy
import actionlib

from drone.msg import MoveForwardAction, MoveForwardResult, MoveForwardFeedback

class MoveForwardServer:
    _feedback = MoveForwardFeedback()
    _result = MoveForwardResult()

    def __init__(self):
        rospy.loginfo("MoveForwardServer begin")
        self.server = actionlib.SimpleActionServer('move_forward', MoveForwardAction, execute_cb=self.execute, auto_start=False)
        self.server.start()

    def execute(self, goal):
        r = rospy.Rate(10)

        rospy.loginfo('Executing, creating power for up %f' % goal.seconds)

        for i in range(int(goal.seconds * 10)):
            rospy.set_param("/up/power", 20)
            t = i  / 10
            self.server.publish_feedback(MoveForwardFeedback(t))
            r.sleep()
        rospy.set_param("/up/power", 0)
        self.server.set_succeeded(MoveForwardResult(True))


if __name__ == '__main__':
    rospy.init_node('move_forward_server')
    server = MoveForwardServer()
    rospy.spin()
