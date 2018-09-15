#!/usr/bin/env python
import serial
import rospy

if __name__ == '__main__':
    rospy.init_node("serial_communication", anonymous=True)

    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(ser.isOpen())
        rate.sleep()
