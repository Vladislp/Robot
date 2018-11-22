#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Int16
import math


class GameLogic():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        self.basket_sub = rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)
        self.servo_pub = rospy.Publisher("servo", Int16, queue_size=10)
        self.listener_sub = rospy.Subscriber("commands", String, self.ref_callback)
        self.ball = None
        self.basket = None
        self.throwing_start_time = None
        self.state = 'idle'
        self.game_started = False
        self.isRounding = False

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

    def thrower_callback(self, speed):
        self.main_board.set_throw(speed.data)

    def ref_callback(self, string):
        if string.data == "START":
            self.game_started = True
        elif string.data == "STOP":
            self.game_started = False

    def spin_once(self):
        center_of_the_field = 340
        very_close = 400

        #self.game_started = True

        if self.ball is None:
            self.rotate()

        elif (self.ball.y > 360 and self.ball.y < 420):
            self.rounding()

            if self.basket is not None:

                if self.basket.x > 290 and self.basket.x < 310:
                    self.stop()
                    self.move_forward()
                    self.thrower(150)

        elif (self.ball.x < 400 and self.ball.x > 340):
            #print("mooving")
            self.move_forward()
        #############################################
        elif (self.ball.x > 194 and self.ball.x < 282):
            self.rotate()
        elif(self.ball.x > 330 and self.ball.x < 450):
            self.move_little_left()
        elif(self.ball.x > 138 and self.ball.x < 310):
            self.rotate()
        elif(self.ball.x > 310 and self.ball.x > 463):
            self.move_little_left()
        #############################################




    def calculate_thrower_speed(self, distance):
        return int(0.0234 * distance + 151.78)

    def move_forward(self):
        self.robot_movement_pub.publish(Point(20, -0, 0))
    def move_little_left(self):
        self.robot_movement_pub.publish(Point(20, 5, 0))

    def move_backward(self):
        self.robot_movement_pub.publish(Point(30, -30, 0))

    def rotate(self):
        self.robot_movement_pub.publish(Point(0, 0, 5))

    def stop(self):
        self.robot_movement_pub.publish(Point(0, 0, 0))

    def thrower(self, speed):
        self.thrower_pub.publish(Int16(speed))

    def right_rounding(self):
        self.robot_movement_pub.publish(Point(-15, 0, -10))

    def left_rounding(self):
        self.robot_movement_pub.publish(Point(15, 0, 10))

    def servo_1(self):
        self.servo_pub.publish(1350)

    def servo_2(self):
        self.servo_pub.publish(1660)

if __name__ == "__main__":
    rospy.init_node('game_logic', anonymous=True)
    rate = rospy.Rate(25)

    game_logic = GameLogic()

    while not rospy.is_shutdown():
        game_logic.spin_once()
        rate.sleep()
