#!/usr/bin/env python
import rospy
import serial
import struct
from drone.msg import Powers


arduino = serial.Serial('/dev/ttyUSB0', 9600, bytesize=serial.EIGHTBITS, timeout=1)
serial_frame = {
    "header": 0x55,
    "len": 8,
    "data": [0, 0, 0, 0, 0, 0, 0, 0],
    "crc": 0
}

rospy.init_node("arduino_serial")
subscriber = rospy.Subscriber("control_data", Powers, callback)

def callback(msg):
    serial_frame["data"][0] = msg.left_alt_power
    serial_frame["data"][1] = msg.right_alt_power
    serial_frame["data"][2] = msg.left_heading_power
    serial_frame["data"][3] = msg.right_heading_power    
    serial_frame["data"][4] = msg.back_alt_power
    serial_frame["data"][5] = msg.light_mode
    serial_frame["data"][6] = int(msg.lock)

    serial_frame["crc"] = 0
    for i in range(serial_frame["len"]):
        serial_frame["crc"] = serial_frame["crc"] ^ (int(serial_frame["data"][i]) & 0b11111111)
    
    packet = struct.pack(
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

    arduino.write(packet)