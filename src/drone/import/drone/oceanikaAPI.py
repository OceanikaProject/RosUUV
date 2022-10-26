#!/usr/bin/env python3

import rospy
import actionlib
from drone.msg import TurnAction, TurnGoal
from drone.msg import DiveAction, DiveGoal


class UUV(object):
    __instance = None

    def __init__(self):
        pass

   
    def __new__(cls, *args, **kwargs):
        if not isinstance(cls.__instance, cls):
            cls.__instance = super(UUV, cls).__new__(cls)
        return cls.__instance

    def move_forward(self):
        pass

    def move_back(self):
        pass

    def turn_left(self):
        client = actionlib.SimpleActionClient('turn', TurnAction)
        client.wait_for_server()
        goal = TurnGoal(target='left')
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def turn_right(self):
        client = actionlib.SimpleActionClient('turn', TurnAction)
        client.wait_for_server()
        goal = TurnGoal(target='right')
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def dive_up(self):
        rospy.init_node('dive_client')
        client = actionlib.SimpleActionClient('dive', DiveAction)
        client.wait_for_server()
        goal = DiveGoal(target='up')
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def dive_down(self):
        rospy.init_node('dive_client')

        client = actionlib.SimpleActionClient('dive', DiveAction)
        client.wait_for_server()
        goal = DiveGoal(target='down')
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def wait(self):
        rospy.sleep(1)

    def blink(self):
        for i in range(6):
            self.lights_on()
            rospy.sleep(.25)
            self.lights_off()
            rospy.sleep(.25)

    def lights_on(self):
        rospy.set_param("/light_mode", 1)

    def lights_off(self):
        rospy.set_param("/light_mode", 0)

    def close_hand(self):
        pass
    
    def open_hand(self):
        pass

    def set_motor_thrust(direction, power):
        if direction == 'up':
            pass
        elif direction == 'forward':
            pass
        elif direction == 'turn'