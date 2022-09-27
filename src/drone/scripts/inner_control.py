#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class PID(object):
    """
    PID regulator algorithm
    """
    def __init__(self, Kp, Ki, Kd):
        self.prev_error = 0
        self.proportional = 0
        self.integral = 0
        self.differential = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    @staticmethod
    def constrain(value):
        if value > 100:
            value = 100
        if value < -100:
            value = -100
        return value

    def regulate(self, current, target, dt):
        """
        current: current value of regulated parameter
        target: target value of regulated paremeter
        dt: sampling period
        return: regulation signal
        """
        error = float(current - target)
        self.proportional = error * self.Kp
        self.integral += error * dt * self.Ki
        self.integral = PID.constrain(self.integral)
        self.differential = (error - self.prev_error) / dt * self.Kd
        self.prev_error = error
        return PID.constrain(self.proportional + self.integral + self.differential)


class ControlSystem(object):
    __instanse = None

    def __new__(cls, *args, **kwargs):
        if not isinstance(cls.__instanse, cls):
            cls.__instanse = super(ControlSystem, cls).__new__(cls)
        return cls.__instanse

    def __del__(self):
        ControlSystem.__instanse = None

    def __init__(self):
        self.__powers = [0, 0, 0, 0, 0]
        self.pid_roll = 0
        self.pid_pitch = 0
        self.pid_yaw = 0

        self.Kp, self.Ki, self.Kd = None, None, None

        self.t = rospy.get_time()
        

    @property
    def powers(self):
        return self.__powers


    @powers.setter
    def powers(self, angles):
        roll, pitch, yaw = angles
        # print(angles)
        dt = rospy.get_time() - self.t
        roll_power = self.pid_roll.regulate(roll, 0, dt) * 100 / 360
        pitch_power = self.pid_pitch.regulate(pitch, 0, dt) * 100 / 180
        yaw_power = self.pid_yaw.regulate(yaw, 0, dt) * 100 / 160

        if roll_power > 100:
            roll_power = 100
        if roll_power < -100:
            roll_power = -100

        if pitch_power > 100:
            pitch_power = 100
        if pitch_power < -100:
            pitch_power = -100

        if yaw_power > 100:
            yaw_power = 100
        if yaw_power < -100:
            yaw_power = -100

       # roll_power /= 2
       # pitch_power /= 2
       # yaw_power /= 2
       # print(roll_power, pitch_power, yaw_power)
        self.__powers = [
            round((pitch_power / 2 - roll_power)),
            round((pitch_power / 2 + roll_power)),
            # round(yaw_power),
            # round(-yaw_power),
            0, 0,
            round(-pitch_power)
        ]

        #self.__powers = [0,0,0,0,0]

        self.t = rospy.get_time()

    @classmethod
    def combine(a, b):
        if a + b == 0:
            return 0
        return a*a/(a+b) + b*b/(a+b)


    def set_pid(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.pid_roll = PID(self.Kp, self.Ki, self.Kd)
        self.pid_pitch = PID(self.Kp, self.Ki, self.Kd)
        self.pid_yaw = PID(self.Kp, self.Ki, self.Kd)



angles = [0, 0, 0]
last_data = [0, 0, 0, 0, 0]
pub = rospy.Publisher('outer_control_topic', Int32MultiArray, queue_size=2)
started = False

autocontrol = None
def callback(data):
    global angles, started, autocontrol
    autocontrol.powers = data.data
    if not started:
        started = True

    # angles = data.data
    # autocontrol.powers = angles
    # msg = Int32MultiArray(data=autocontrol.powers)
    # pub.publish(msg)


def timer_callback(event):
    global started, pub, last_data
    if started:
        msg = Int32MultiArray(data=autocontrol.powers)
        pub.publish(msg) 


def move():
    rospy.init_node('outer_control_topic')
    global autocontrol

    autocontrol = ControlSystem()
    autocontrol.set_pid(5, 0, 0)


    rospy.Subscriber("talker_topic", Float32MultiArray, callback)
    timer = rospy.Timer(rospy.Duration(0.01), timer_callback)

    rospy.loginfo("inner control started")
    rospy.spin()
    timer.shutdown()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass