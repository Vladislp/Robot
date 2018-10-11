#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class GameLogic():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.callback)
        #self.basket_sub = rospy.Subscriber("basket_coordinates", Point, self.callback)
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.ball = None
        #self.basket = None

    def callback(self, point):
        self.ball = point

    def spin_once(self):
        center = 240

        if self.ball is None:
            self.rotate()
        elif (self.ball.y > 160 and self.ball.y < 180):
            self.rounding()

    def move_forward(self):
        self.robot_movement_pub.publish(Point(40, -90, 0))

    def move_backward(self):
        self.robot_movement_pub.publish(Point(20, 90, 0))

    def rotate(self):
        self.robot_movement_pub.publish(Point(0, 0, 10))

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
