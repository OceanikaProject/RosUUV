#!/usr/bin/env python3

import rospy
import actionlib
import sys
from drone.msg import DiveAction, DiveResult, DiveFeedback
# from drone import PID
from geometry_msgs.msg import PoseStamped

class PID:

    def __init__(self, Kp, Ki, Kd):
        self.prev_error = 0
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = 0

    @staticmethod
    def constrain(value, lower_limit, higher_limit):
        if value < lower_limit: value = lower_limit
        if value > higher_limit: value = higher_limit
        return value

    def control(self, current, dt):
        dt = dt.to_sec()
        error = float(current - self.target)
        self.proportional = error * self.Kp
        self.integral += PID.constrain(error * dt * self.Ki, -100, 100)
        self.derivative = (error - self.prev_error) / dt * self.Kd
        self.prev_error = error
        return int(PID.constrain(self.proportional + self.integral + self.derivative, -100, 100))

    def break_pid(self):
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    def set_target(self, target):
        if target != self.target:
            self.break_pid
        self.target = target

    def set_gains(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D


class DiveActionServer:
    _feedback = DiveFeedback()
    _result = DiveResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('dive', DiveAction, execute_cb=self.execute, auto_start=False)
        self.server.start()

    def execute(self, goal):
        def cb(msg):
            self.depth = msg.pose.position.z
            print(self.depth)
        sub = rospy.Subscriber("navigation_module", PoseStamped, cb)
        
        r = rospy.Rate(10)
        r.sleep()
        depth_gains = rospy.get_param('PidDepth')
        pid = PID(depth_gains['P'], depth_gains['I'], depth_gains['D'])
        pid.set_target(goal.target)

        rospy.loginfo('Get target depth - %f (%f)' % (goal.target, self.depth))

        t = rospy.Time.now()

        while not (self.depth - 0.05 < goal.target < self.depth + 0.05 or rospy.is_shutdown()):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                break
            dt = rospy.Time.now() - t
            t = rospy.Time.now()
            print(dt, dt.to_sec())
            power = pid.control(self.depth, dt)
            rospy.loginfo('%f' % power)

            rospy.set_param("/up/power", power)
            self.server.publish_feedback(DiveFeedback(self.depth))
            r.sleep()
        rospy.set_param("/up/power", 0)
        self.server.set_succeeded(DiveResult(self.depth))
        sub.unregister()

if __name__ == "__main__":
    rospy.init_node("dive_server")
    server = DiveActionServer()
    rospy.spin()
