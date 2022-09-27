#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from drone.msg import DoDishesAction, DoDishesResult, DoDishesFeedback

class DoDishesServer:
    _feedback = DoDishesFeedback()
    _result = DoDishesResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        r = rospy.Rate(1)
        self.clear_dishes = 0

        for i in range(1, goal.dishes+1):
            self.server.publish_feedback(DoDishesFeedback(i))
            self.clear_dishes+=1
            r.sleep()
        self.server.set_succeeded(DoDishesResult(self.clear_dishes))


if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = DoDishesServer()
    rospy.spin()
