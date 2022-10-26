#!/usr/bin/env python3

import rospy
import actionlib
import sys
from drone.msg import DiveAction, DiveResult, DiveFeedback
from drone.PID import PID
from geometry_msgs.msg import PoseStamped


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
        

        target = 0
        if goal.target == 'up':
            target = self.depth + 0.2
            print(target)
        elif goal.target == 'down':
            target = self.depth - 0.2
            print(target)
        pid.set_target(target)

        
        # pid.set_target(goal.target)

        rospy.loginfo('Get target depth - %s (%f)' % (goal.target, self.depth))

        t = rospy.Time.now()

        while not (self.depth - 0.05 < target < self.depth + 0.05 or rospy.is_shutdown()):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                break
            dt = rospy.Time.now() - t
            t = rospy.Time.now()
            print(dt, dt.to_sec())
            power = pid.control(self.depth, dt)
            rospy.loginfo('%f' % power)

            # rospy.set_param("/up/power", power)
            rospy.set_param("/vertical_left", power)
            rospy.set_param("/vertical_right", power)
            rospy.set_param("/vertical_back", power)
            self.server.publish_feedback(DiveFeedback(self.depth))
            r.sleep()
        # rospy.set_param("/up/power", 0)
        rospy.set_param("/vertical_left", 0)
        rospy.set_param("/vertical_right", 0)
        rospy.set_param("/vertical_back", 0)
        self.server.set_succeeded(DiveResult(self.depth))
        sub.unregister()

if __name__ == "__main__":
    rospy.init_node("dive_server")
    server = DiveActionServer()
    rospy.spin()
