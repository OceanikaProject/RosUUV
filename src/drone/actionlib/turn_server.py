#!/usr/bin/env python3

import rospy
import actionlib
import sys
from drone.msg import TurnAction, TurnResult, TurnFeedback
# from drone import PID
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

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


class TurnActionServer:
    _feedback = TurnFeedback()
    _result = TurnResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('turn', TurnAction, execute_cb=self.execute, auto_start=False)
        self.server.start()

    def execute(self, goal):
        def cb(msg):
            _, _, self.yaw = euler_from_quaternion([
           navigation_msg.pose.orientation.x,
           navigation_msg.pose.orientation.y,
           navigation_msg.pose.orientation.z,
           navigation_msg.pose.orientation.w
        ])
            print(self.yaw)
        sub = rospy.Subscriber("navigation_module", PoseStamped, cb)
        
        r = rospy.Rate(10)
        r.sleep()
        yaw_gains = rospy.get_param('PidYaw')
        pid = PID(yaw_gains['P'], yaw_gains['I'], yaw_gains['D'])

        self.yaw -= 180
        pid.set_target(0)
        target = self.yaw
        if goal.target == 'left':
            target -= 90
        elif goal.target == 'right':
            target += 90

        rospy.loginfo('Get target yaw - %s (%f)' % (goal.target, self.yaw))

        t = rospy.Time.now()

        while not (self.yaw - 3 < goal.target < self.yaw + 3 or rospy.is_shutdown()):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                break
            dt = rospy.Time.now() - t
            t = rospy.Time.now()
            print(dt, dt.to_sec())
            if self.yaw - target < -180:
                power = pid.control(self.yaw - target + 360, dt)   
            elif self.yaw - target > 180:
                power = pid.control(self.yaw - target - 360, dt)
            else:      
                power = pid.control(self.yaw, dt)
            rospy.loginfo('%f' % power)

            rospy.set_param("/turn/power", power)
            self.server.publish_feedback(TurnFeedback(self.yaw))
            r.sleep()
        rospy.set_param("/turn/power", 0)
        self.server.set_succeeded(TurnResult(self.yaw))
        sub.unregister()

if __name__ == "__main__":
    rospy.init_node("turn_server")
    server = TurnActionServer()
    rospy.spin()
