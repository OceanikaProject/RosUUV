#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from RPi import GPIO
import pigpio


GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()


def callback(power):
    pi.set_servo_pulsewidth(13, power.data)
    print("callback", power.data)


if __name__ == "__main__":
    rospy.init_node("hand")
    subscriber = rospy.Subscriber("hand_angle", Int16, callback)
    rospy.spin()
    GPIO.cleanup()
