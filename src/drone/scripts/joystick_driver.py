#!/usr/bin/env python
import rospy
from drone.msg import Joystick
import socket


class JoystickRepeater:
    ROS_IP = "192.168.1.114"
    PORT = 1234

    def __init__(self):
        rospy.init_node("joystick_driver_node")
        self.pub = rospy.Publisher('joystick_state', Joystick, queue_size=2)
        self.msg = Joystick()



        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("192.168.1.114", 1234))

    def parse(self, esp8266_data):
        print(esp8266_data[0])
        if esp8266_data[0] == 0x77:
            self.msg.left_x = esp8266_data[4] | esp8266_data[5] << 8
            self.msg.left_y = esp8266_data[6] | esp8266_data[7] << 8
            self.msg.right_x = esp8266_data[8] | esp8266_data[9] << 8
            self.msg.right_y = esp8266_data[10] | esp8266_data[11] << 8

            self.msg.Short.A = esp8266_data[14]
            self.msg.Short.B = esp8266_data[15]
            self.msg.Short.C = esp8266_data[16]
            self.msg.Short.D = esp8266_data[17]
            self.msg.Short.Lsw = esp8266_data[18]
            self.msg.Short.Rsw = esp8266_data[19]
            self.msg.Short.HOLD = esp8266_data[20]
            self.msg.Short.LED = esp8266_data[21]
            self.msg.Short.L = esp8266_data[22]
            self.msg.Short.R = esp8266_data[23]

            self.msg.Long.A = esp8266_data[24]
            self.msg.Long.B = esp8266_data[25]
            self.msg.Long.C = esp8266_data[26]
            self.msg.Long.D = esp8266_data[27]
            self.msg.Long.Lsw = esp8266_data[28]
            self.msg.Long.Rsw = esp8266_data[29]
            self.msg.Long.HOLD = esp8266_data[30]
            self.msg.Long.LED = esp8266_data[31]
            self.msg.Long.L = esp8266_data[32]
            self.msg.Long.R = esp8266_data[33]

            self.msg.pressed_buttons = esp8266_data[34] | esp8266_data[35] << 8
            self.msg.State = esp8266_data[36]
            self.msg.Battery = esp8266_data[12] | esp8266_data[13] << 8
            self.msg.SpeedMode = esp8266_data[37]
            self.msg.LightMode = esp8266_data[38]
            print_msg = [
                self.msg.left_x,
                self.msg.right_x,
                self.msg.right_y,
                self.msg.left_y,
                self.msg.Battery,
                self.msg.Short,
                self.msg.Long,
                self.msg.pressed_buttons,
                self.msg.State,
                self.msg.SpeedMode,
                self.msg.LightMode
            ]
            print(print_msg)
            return True
        return False

    def run(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            print("waiting...")
            data, addr = self.sock.recvfrom(1024)
            data = [byte for byte in data]
            print(data)
            if self.parse(data):
                self.pub.publish(self.msg)
            rate.sleep()


if __name__ == '__main__':
    repeater = JoystickRepeater()
    try:
        repeater.run()
    except rospy.ROSInterruptException:
        pass
