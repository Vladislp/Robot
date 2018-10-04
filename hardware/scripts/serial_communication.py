#!/usr/bin/env python

import rospy
from hardware.MainBoard import ComportMainboard
from geometry_msgs.msg import Point


class SerialCommunication():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.callback)
        self.main_board = ComportMainboard()
        self.main_board.run()
        self.point = None

    def callback(self, point):
        center = 320
        self.point = point.x

    def spin_once(self):
        center = 320
        if self.point is None:
            self.main_board.launch_motor(10, 10, 10, 0)
        elif center + 20 > self.point > center - 20:
            self.main_board.launch_motor(0, -10, 10, 0)


if __name__ == '__main__':
    rospy.init_node('serial_communication', anonymous=True)
    rate = rospy.Rate(25)
    serial_communication = SerialCommunication()

    while not rospy.is_shutdown():
        serial_communication.spin_once()
        rate.sleep()
