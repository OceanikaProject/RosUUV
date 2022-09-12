#!/usr/bin/env python

from __future__ import print_function
from drone.srv import MoveForward
import rospy


def move(req):
    print("Returning [%s | %s]"%(req.power, req.duration))
    rospy.set_param("/forward/power", req.power)
    rospy.sleep(req.duration)
    rospy.set_param("/forward/power", 0)


def move_forward_server():
    rospy.init_node("move_forward_server")
    s = rospy.Service('move_forward', MoveForward, move)
    print("ready to move forward")
    rospy.spin()

if __name__ == "__main__":
    move_forward_server()