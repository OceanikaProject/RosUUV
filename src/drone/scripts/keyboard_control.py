#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32MultiArray
import sys
import select
import tty
import termios


settings = termios.tcgetattr(sys.stdin)

def get_key(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def move():
    # rospy.Subscribe('user_input', Int32MultiArray, callback)
    forward = 0
    roll = 0
    pitch = 0
    yaw = 0

    pub = rospy.Publisher('outer_control_topic', Int32MultiArray, queue_size=2)
    rospy.init_node('outer_control_node')
    rate = rospy.Rate(1000)
    rospy.loginfo("outer control started")
    data = [0, forward, forward, 0, 0]
    while not rospy.is_shutdown():
        # k = ord(getch.getch())
        k = get_key(0.1)
        # print(k)
        if k == '\x03':
            break
        if (k == 'w' or k == 'W') and pitch < 100:
            pitch += 1       
        elif (k == 's' or k == 'S') and pitch > -100:
            pitch -= 1
        else:
            if pitch > 0:
                pitch -= 1
            if pitch < 0:
                pitch += 1
        
        if (k == 'q' or k == 'Q') and roll < 100:
            roll += 1       
        elif (k == 'e' or k == 'E') and roll > -100:
            roll -= 1
        else:
            if roll > 0:
                roll -= 1
            if roll < 0:
                roll += 1

        if (k == 'a' or k == 'A') and yaw < 100:
            yaw += 1       
        elif (k == 'd' or k == 'D') and yaw > -100:
            yaw -= 1
        else:
            if yaw > 0:
                yaw -= 1
            if yaw < 0:
                yaw += 1

        if (k == 'z' or k == 'Z') and forward < 100:
            forward += 1       
        elif (k == 'c' or k == 'C') and forward > -100:
            forward -= 1

    
        def combine(a, b):
            if a + b == 0:
                return 0
            return a*a/(a+b) + b*b/(a+b)
                

        data = [
            combine(pitch / 2, roll), # left alt 
            combine(forward, yaw), # left head
            combine(forward, -yaw),  # right head
            combine(pitch / 2, -roll), # right alt
            -pitch # back alt
        ]
        msg = Int32MultiArray(data=data)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
        
