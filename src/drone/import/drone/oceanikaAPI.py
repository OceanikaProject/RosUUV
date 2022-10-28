#!/usr/bin/env python3

import rospy
import actionlib
from drone.msg import TurnAction, TurnGoal
from drone.msg import DiveAction, DiveGoal
from drone.srv import MoveForward


class UUV(object):
    __instance = None

    def __init__(self):
        pass
   
    def __new__(cls, *args, **kwargs):
        if not isinstance(cls.__instance, cls):
            cls.__instance = super(UUV, cls).__new__(cls)
        return cls.__instance

    def move_forward(self):
        rospy.wait_for_service('move_forward')
        try:
            move_forward = rospy.ServiceProxy('move_forward', MoveForward)
            move_forward(30, 3)
        except rospy.ServiceException as e:
            rospy.LOGERR(f'Service call failed: %{e}')

    def move_back(self):
        rospy.wait_for_service('move_forward')
        try:
            move_forward = rospy.ServiceProxy('move_forward', MoveForward)
            move_forward(-30, 3)
        except rospy.ServiceException as e:
            rospy.LOGERR(f'Service call failed: %{e}')

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
        rospy.set_param("/hand/mode", 1)
        rospy.sleep(2)
        rospy.set_param("/hand/mode", 0)
    
    def open_hand(self):
        rospy.set_param("/hand/mode", 2)
        rospy.sleep(2)
        rospy.set_param("/hand/mode", 0)

    def set_vertical_left_motor_thrust(self, power):
        if type(power) not in [int, float]:
            raise ValueError(f"Value must be int or float, get instead %{type(power)}")
        power = round(power)
        if power > 60:
            power = 60
        elif power < -60:
            power = -60
        rospy.set_param("/vertical_left/power", power)

    def set_vertical_right_motor_thrust(self, power):
        if type(power) not in [int, float]:
            raise ValueError(f"Value must be int or float, get instead %{type(power)}")
        power = round(power)
        if power > 60:
            power = 60
        elif power < -60:
            power = -60
        rospy.set_param("/vertical_right/power", power)

    def set_horizontal_left_motor_thrust(self, power):
        if type(power) not in [int, float]:
            raise ValueError(f"Value must be int or float, get instead %{type(power)}")
        power = round(power)
        if power > 60:
            power = 60
        elif power < -60:
            power = -60
        rospy.set_param("/horizontal_left/power", power)

    def set_horizontal_right_motor_thrust(self, power):
        if type(power) not in [int, float]:
            raise ValueError(f"Value must be int or float, get instead %{type(power)}")
        power = round(power)
        if power > 60:
            power = 60
        elif power < -60:
            power = -60
        rospy.set_param("/horizontal_right/power", power)

    def set_vertical_back_motor_thrust(self, power):
        if type(power) not in [int, float]:
            raise ValueError(f"Value must be int or float, get instead %{type(power)}")
        power = round(power)
        if power > 60:
            power = 60
        elif power < -60:
            power = -60
        rospy.set_param("/vertical_back/power", power)