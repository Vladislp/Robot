#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError


class GameLogic():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        self.basket_sub = rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.ball = None
        self.basket = None

    def ball_callback(self, point):
        if point.x < 0:
            self.ball = None
        else:
            self.ball = point

    def basket_callback(self, point):
        if point.x < 0:
            self.basket = point
        else:
            self.basket = None

    def spin_once(self):
        center_of_the_field = 340
        very_close = 400


        if self.ball is None:
            print("rotating")
            self.rotate()
        elif (self.ball.x < 400 and self.ball.x > 340):
            print("mooving")
            self.move_forward()



    def move_forward(self):
        self.robot_movement_pub.publish(Point(20, 3, 0))

    def move_backward(self):
        self.robot_movement_pub.publish(Point(30, -30, 0))

    def rotate(self):
        self.robot_movement_pub.publish(Point(0, 0, 5))

    def stop(self):
        self.robot_movement_pub.publish(Point(0, 0, 0))

    def rounding(self):
        self.robot_movement_pub.publish(Point(10, 0, 20))

if __name__ == "__main__":
    rospy.init_node('game_logic', anonymous=True)
    rate = rospy.Rate(25)

    game_logic = GameLogic()

    while not rospy.is_shutdown():
        game_logic.spin_once()
        rate.sleep()
