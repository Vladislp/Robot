#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class GameLogic():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.callback)
        self.robot_movement_pub = rospy.Publisher('robot_movement')
        self.ball_points = None

    def callback(self, point):
        self.ball_points = point

    def spin_once(self):
        center = 320

    


if __name__ == "__main__":
    rospy.init_node('Game_Logic_Node', anonymous=True)
    rate = rospy.Rate(25)

    game_logic = GameLogic()

    while not rospy.is_shutdown():
        rate.sleep()
