#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node("serial_communication", anonymous=True)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        print("Hello")
        rate.sleep()
