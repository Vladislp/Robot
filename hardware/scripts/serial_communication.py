#!/usr/bin/env python
import serial
import rospy
import time

if __name__ == '__main__':
    rospy.init_node("serial_communication", anonymous=True)

    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if ser.isOpen():
            ##ser.write("sd:0:-40:40:40\n")
            ser.write("d:1200\n")
            ##time.sleep(10)
            ##ser.write("sd:0:0:0:0\n")
            print("working")
        rate.sleep()
