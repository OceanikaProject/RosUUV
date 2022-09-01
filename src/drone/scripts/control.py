#!/usr/bin/env python
import rospy
import message_filters
import serial
import struct
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from drone.msg import Joystick, Powers
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
        return PID.constrain(self.proportional + self.integral + self.derivative, -100, 100)

    def set_target(self, target):
        self.target = target


class Control:

    def __init__(self):
        rospy.init_node("control")

        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, bytesize=serial.EIGHTBITS, timeout=1)
        self.serial_frame = {
            "header": 0x55,
            "len": 8,
            "data": [0, 0, 0, 0, 0, 0, 0, 0],
            "crc": 0
        }

        self.pub_hand = rospy.Publisher("/hand_angle", Int16, queue_size=1)
        self.powers_msg = Powers()
        self.power_hand = Int16()
        # rospy.Subscriber("esp", Joystick, self.control_callback)
        joystick_subscriber = message_filters.Subscriber("joystick_state", Joystick)
        navigation_subscriber = message_filters.Subscriber("talker_topic", PoseStamped)
        self.ts = message_filters.TimeSynchronizer([joystick_subscriber, navigation_subscriber], 10)
        self.ts.registerCallback(self.control_callback)
        # self.pid_roll = PID(2, 0, 0)

        pitch_gains = rospy.get_param('PidPitch')
        depth_gains = rospy.get_param('PidDepth')
        self.pid_pitch = PID(pitch_gains['P'], pitch_gains['I'], pitch_gains['D'])
        # self.pid_yaw = PID(2, 0, 0)
        self.pid_depth = PID(depth_gains['P'], depth_gains['I'], depth_gains['D'])

        self.oceanic_yaml = "/home/ubuntu/drone_ros/src/drone/config/oceanic.yaml"
        self.pid_yaml = "/home/ubuntu/drone_ros/src/drone/config/pid.yaml"

        self.t = rospy.Time.now()

    def save_yaml(self):
        import yaml

        with open(self.oceanic_yaml, 'r') as f:
            oceanic = yaml.safe_load(f)

        oceanic["vertical_left"]["reversed"] = rospy.get_param("/vertical_left")["reversed"]
        oceanic["vertical_right"]["reversed"] = rospy.get_param("/vertical_right")["reversed"]
        oceanic["horizontal_left"]["reversed"] = rospy.get_param("/horizontal_left")["reversed"]
        oceanic["horizontal_right"]["reversed"] = rospy.get_param("/horizontal_right")["reversed"]
        oceanic["vertical_back"]["reversed"] = rospy.get_param("/vertical_back")["reversed"]

        with open(self.oceanic_yaml, 'w') as f:
            yaml.dump(oceanic, f, default_flow_style=False)

        with open(self.pid_yaml, 'r') as f:
            pid = yaml.safe_load(f)

        pid["PidPitch"]["P"] = rospy.get_param("/PidPitch")["P"]
        pid["PidPitch"]["I"] = rospy.get_param("/PidPitch")["I"]
        pid["PidPitch"]["D"] = rospy.get_param("/PidPitch")["D"]
        pid["PidDepth"]["P"] = rospy.get_param("/PidDepth")["P"]
        pid["PidDepth"]["I"] = rospy.get_param("/PidDepth")["I"]
        pid["PidDepth"]["D"] = rospy.get_param("/PidDepth")["D"]

        with open(self.pid_yaml, 'w') as f:
            yaml.dump(pid, f, default_flow_style=False)


    @staticmethod
    def map(x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def control_callback(self, joystick_msg, navigation_msg):

        short_buttons = joystick_msg.Short
        long_buttons = joystick_msg.Long
        pressed_buttons = joystick_msg.pressed_buttons

        A_active = bool((pressed_buttons >> 0) & 0b0000000000000001)
        D_active = bool((pressed_buttons >> 3) & 0b0000000000000001)
        R_active = bool((pressed_buttons >> 8) & 0b0000000000000001)
        L_active = bool((pressed_buttons >> 7) & 0b0000000000000001)

        if A_active:
            self.power_hand.data = 1148
        elif D_active:
            self.power_hand.data = 1832
        else:
            self.power_hand.data = 1488

        speed_mode = joystick_msg.SpeedMode
        light_mode = joystick_msg.LightMode

        lx = Control.map(joystick_msg.left_x, 0, 1023, -100, 100)
        ly = Control.map(joystick_msg.left_y, 0, 1023, -100, 100)
        rx = Control.map(joystick_msg.right_x, 0, 1023, -100, 100)
        ry = Control.map(joystick_msg.right_y, 0, 1023, -100, 100)

        divisor = 1
        if speed_mode == 0:
            divisor = 0.3
        elif speed_mode == 1:
            divisor = 0.6
        elif speed_mode == 2:
            divisor = 1

        stabilization_on = not (joystick_msg.State & 0b00000001)
        self.powers_msg.lock = (joystick_msg.State & 0b00000010) >> 1
        self.serial_frame["data"][6] = self.powers_msg.lock
        yaw_on = (joystick_msg.State & 0b00001000) >> 3
        tangage_on = (joystick_msg.State & 0b00010000) >> 4

        lx = int(lx * divisor) if tangage_on else 0
        ly = int(ly * divisor)
        rx = int(rx * divisor) if yaw_on else 0
        ry = int(ry * divisor)

        # State
        # 00000000 - 0 - lock on, stabilization off, tangage off
        # 00000001 - 1 - lock on, stabilization on, tangage off
        # 00000010 - 2 - lock off, stabilization off, tangage off
        # 00000011 - 3 - lock off, stabilization on, tangage off
        # 00010000 - 16 - lock on, stabilization off, tangage on
        # 00010001 - 17 - lock on, stabilization on, tangage on
        # 00010010 - 18 - lock off, stabilization off, tangage on
        # 00010011 - 19 - lock off, stabilization on, tangage on

        if light_mode == 0:
            self.powers_msg.Light_pwm = 0
        elif light_mode == 1:
            self.powers_msg.Light_pwm = 1000
        elif light_mode == 2:
            self.powers_msg.Light_pwm = 1500
        elif light_mode == 3:
            self.powers_msg.Light_pwm = 2000

        roll, pitch, yaw = euler_from_quaternion([
            navigation_msg.pose.orientation.x,
            navigation_msg.pose.orientation.y,
            navigation_msg.pose.orientation.z,
            navigation_msg.pose.orientation.w
        ])

        roll = roll * 57.2958
        pitch = pitch * 57.2958
        yaw = yaw * 57.2958

        # print(roll, pitch, yaw)

        dt = rospy.Time.now() - self.t


        vertical_left_params = rospy.get_param('vertical_left')
        vertical_right_params = rospy.get_param('vertical_right')
        horizontal_left_params = rospy.get_param('horizontal_left')
        horizontal_right_params = rospy.get_param('horizontal_right')
        vertical_back_params = rospy.get_param('vertical_back')

        vertical_left_direction = -1 if vertical_left_params["reversed"] else 1
        vertical_right_direction = -1 if vertical_right_params["reversed"] else 1
        horizontal_left_direction = -1 if horizontal_left_params["reversed"] else 1
        horizontal_right_direction = -1 if horizontal_right_params["reversed"] else 1
        vertical_back_direction = -1 if vertical_back_params["reversed"] else 1



        if stabilization_on:
            if lx in range(-5, 6) and ly in range(-5, 6):
                pitch_power = self.pid_pitch.control(pitch, dt)
                depth_power = self.pid_depth.control(navigation_msg.pose.position.z, dt)

                left_alt_pwm = depth_power / 2
                right_alt_pwm = depth_power / 2
                back_alt_pwm = depth_power

                self.serial_frame["data"][0] = left_alt_pwm
                self.serial_frame["data"][1] = right_alt_pwm
                self.serial_frame["data"][4] = back_alt_pwm
            else:
                left_alt_pwm = ly / 2 + lx / 2
                right_alt_pwm = ly / 2 + lx / 2
                back_alt_pwm = ly - lx

                self.serial_frame["data"][0] = left_alt_pwm
                self.serial_frame["data"][1] = right_alt_pwm
                self.serial_frame["data"][4] = back_alt_pwm

                # self.pid_roll.set_target(roll)
                self.pid_pitch.set_target(pitch)
                # self.pid_yaw.set_target(yaw)
                self.pid_depth.set_target(navigation_msg.pose.position.z)

        else:
            left_alt_pwm = ly / 2 + lx / 2
            right_alt_pwm = ly / 2 + lx / 2
            back_alt_pwm = ly - lx

            self.serial_frame["data"][0] = left_alt_pwm
            self.serial_frame["data"][1] = right_alt_pwm
            self.serial_frame["data"][4] = back_alt_pwm

        left_heading_pwm = ry + rx
        right_heading_pwm = ry - rx

        self.serial_frame["data"][2] = left_heading_pwm
        self.serial_frame["data"][3] = right_heading_pwm

        self.serial_frame["data"][0] = int(self.serial_frame["data"][0]) * vertical_right_direction # d3  motor3
        self.serial_frame["data"][1] = int(self.serial_frame["data"][1]) * vertical_left_direction # d5  motor4
        self.serial_frame["data"][2] = int(self.serial_frame["data"][2]) * horizontal_right_direction # d6  motor1
        self.serial_frame["data"][3] = int(self.serial_frame["data"][3]) * horizontal_left_direction # d9  motor2
        self.serial_frame["data"][4] = int(self.serial_frame["data"][4]) * vertical_back_direction # d10 motor5

        print(self.serial_frame["data"])

        self.t = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.serial_frame["crc"] = 0
            for i in range(self.serial_frame["len"]):
                self.serial_frame["crc"] = self.serial_frame["crc"] ^ (int(self.serial_frame["data"][i]) & 0b11111111)
            print(self.serial_frame["crc"])
            msg = struct.pack(
                "2B8b1B",
                self.serial_frame["header"],
                self.serial_frame["len"],
                self.serial_frame["data"][0],
                self.serial_frame["data"][1],
                self.serial_frame["data"][2],
                self.serial_frame["data"][3],
                self.serial_frame["data"][4],
                self.serial_frame["data"][5],
                self.serial_frame["data"][6],
                self.serial_frame["data"][7],
                self.serial_frame["crc"]
            )
            self.arduino.write(msg)
            self.pub_hand.publish(self.power_hand)
            rate.sleep()
        self.save_yaml()
        # rospy.spin()


if __name__ == '__main__':
    ctrl = Control()
    try:
        rospy.loginfo("START")
        ctrl.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("FINISH")
        import yaml

        with open('/home/ubuntu/drone_ros/src/drone/config/oceanic.yaml', 'r') as f:
            oceanic_yaml = yaml.safe_load(f)
        
        oceanic_yaml['vertical_left']['reversed'] = rospy.get_param('vertical_left')['reversed']

        with open('/home/ubuntu/drone_ros/src/drone/config/oceanic.yaml', 'w') as f:
            yaml.dump(oceanic_yaml, f, default_flow_style=False)



