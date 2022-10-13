#!/usr/bin/env python3
import rospy
import serial
import struct
from std_msgs.msg import Int16
# from drone.PID import PID


class Autocontrol:

    def __init__(self):
        rospy.init_node("autocontrol")

        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, bytesize=serial.EIGHTBITS, timeout=1)
        self.serial_frame = {
            "header": 0x55,
            "len": 8,
            "data": [0, 0, 0, 0, 0, 0, 0, 0],
            "crc": 0
        }
        self.pub_hand = rospy.Publisher("/hand_angle", Int16, queue_size=1)


    @staticmethod
    def constrain(x, higher_limit, lower_limit):
        if x > higher_limit: x = higher_limit
        if x < lower_limit: x = lower_limit
        return x


    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            forward_power = Autocontrol.constrain(rospy.get_param("forward")["power"], 100, -100)
            up_power = Autocontrol.constrain(rospy.get_param("up")["power"], 100, -100)
            light_mode = rospy.get_param("light_mode")
            # speed_mode = rospy.get_param("speed_mode")
            # turn_power = Autocontrol.constrain(rospy.get_param("turn")["power"], 100, -100)
            self.serial_frame["data"][0] = up_power
            self.serial_frame["data"][1] = forward_power
            self.serial_frame["data"][2] = forward_power
            self.serial_frame["data"][3] = up_power
            self.serial_frame["data"][4] = up_power
            self.serial_frame["data"][5] = light_mode
            # self.serial_frame["data"][6]

            self.serial_frame["crc"] = 0
            for i in range(self.serial_frame["len"]):
                self.serial_frame["crc"] = self.serial_frame["crc"] ^ (int(self.serial_frame["data"][i]) & 0b11111111)

            print(self.serial_frame["data"])
            msg = struct.pack(
                "2B5b4B",
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
            self.pub_hand.publish(1484)
            rate.sleep()
        
    
if __name__ == "__main__":
    ctrl = Autocontrol()
    try:
        rospy.loginfo("START")
        ctrl.run()
        
    except rospy.ROSInterruptException:
        pass    



