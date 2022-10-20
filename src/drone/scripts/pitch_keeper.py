#!/usr/bin/env python3

import rospy
from drone.PID import PID
from geometry_msgs.msg import Vector3


class PitchKeeper:
    
    def __init__(self):
        gains = rospy.get_param('PidPitch')
        self.pid = PID(gains['P'], gains['I'], gains['D'], -40, 40)
        self.sub = rospy.Subscriber("euler", Vector3, self.callback)
        self.t = rospy.Time.now()
        self.pid.set_target(0)
    
    def callback(self, msg):
        gains = rospy.get_param('PidPitch')
        self.pid.set_gains(gains['P'], gains['I'], gains['D'])

        dt = rospy.Time.now() - self.t
        power = self.pid.control(msg.y, dt)
        rospy.set_param("/up/power", power)    

if __name__ == "__main__":
    rospy.init_node("pitch_keeper")
    keeper = PitchKeeper()
    rospy.spin()

