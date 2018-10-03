#!/usr/bin/env python

import rospy
from hardware.MainBoard import MainBoard
from geometry_msgs.msg import Point


class Serial_Communication():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.callback)
        self.main_board = MainBoard()
        self.main_board.run()
        self.point = None

    def callback(self, point):
        center = 320
        self.point = point.x

    def spin_once(self):
        center = 320
        if self.point == None:
            self.main_board.launch_motor(10, 10, 10, 0)
        elif (self.point < center + 20 and self.point > center - 20):
            self.main_board.launch_motor(0, -10, 10, 0)

if __name__ == '__main__':
    rospy.init_node('serial_communication', anonymous=True)
    rate = rospy.Rate(2) # 2hz
    serial_communication = Serial_Communication()

    while not rospy.is_shutdown():
        serial_communication.spin_once()
        rate.sleep()