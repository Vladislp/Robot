#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point

class GameLogic():

    def __init__(self):
        self.sub = rospy.Subscriber("ball_coordinates", Point, self.callback)

    def callback(self, point):
        pass

if __name__ == "__main__":
    rospy.init_node('Game_Logic_Node', anonymous=True)
    rate = rospy.Rate(2)

    game_logic = GameLogic()

    while not rospy.is_shutdown():
        rate.sleep()