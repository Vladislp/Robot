#! /usr/bin/env python

import rospy
from hardware.MainBoard import ComportMainboard
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import math

WHEEL_ONE_ANGLE = 120
WHEEL_TWO_ANGLE = 60
WHEEL_THREE_ANGLE = 270
WHEEL_DISTANCE_FROM_CENTER = 0.133
ROBOT_SPEED = 30
ROBOT_TURN_SPEED = 50

middle = None


class SerialCommunication():

    def __init__(self):
        self.sub = rospy.Subscriber("robot_movement", Point, self.callback)
        self.main_board = ComportMainboard()
        self.main_board.run()
        # self.ball_point = None

        self.wheel_one_speed = 0
        self.wheel_two_speed = 0
        self.wheel_three_speed = 0
        ###########################################

    def get_speed_for_wheel(self, wheel_angle, drive_angle, robot_speed, wheel_distance_from_center,
                            robot_angular_velocity):
        move_speed = robot_speed * math.cos(
            math.radians(drive_angle - wheel_angle) + wheel_distance_from_center * robot_angular_velocity)
        turn_speed = wheel_distance_from_center - robot_angular_velocity
        return move_speed + turn_speed

    def move(com,move_speed):
        com.write("sd:{}:{}:{}".format(move_speed[0], move_speed[1], move_speed[2]))
        com.Readmsgs()

    def move_command(move_speed):
        return "sd:{}:{}:{}".format(move_speed[0], move_speed[3], move_speed[2])

    def set_movement(self, linear_speed, direction_degrees, angular_speed):
        wheel1 = self.get_speed_for_wheel(WHEEL_ONE_ANGLE, direction_degrees, linear_speed, WHEEL_DISTANCE_FROM_CENTER,
                                          angular_speed)
        wheel2 = self.get_speed_for_wheel(WHEEL_TWO_ANGLE, direction_degrees, linear_speed, WHEEL_DISTANCE_FROM_CENTER,
                                          angular_speed)
        wheel3 = self.get_speed_for_wheel(WHEEL_THREE_ANGLE, direction_degrees, linear_speed,
                                          WHEEL_DISTANCE_FROM_CENTER, angular_speed)
        self.define_wheels(round(wheel1, 2), round(wheel2, 2), round(wheel3, 2))

    def callback(self, point):
        self.set_movement(point.x, point.y, point.z)

    def define_wheels(self, wheel1, wheel2, wheel3):
        self.wheel_one_speed = wheel1
        self.wheel_two_speed = wheel2
        self.wheel_three_speed = wheel3
        self.main_board.launch_motor(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed, 0)

    # def define_wheels(self, wheel1, wheel2, wheel3):
    # if (340 > middle > 300):
    # self.wheel_one_speed = 0
    # self.wheel_two_speed = 0
    # self.wheel_three_speed = 0

    # else:

    # self.wheel_one_speed = 10
    # self.wheel_two_speed = 10
    # self.wheel_three_speed = 10

    # self.main_board.launch_motor(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed, 0)

    ###########################################

    # def callback(self, point):
    # self.prepare_movement(point.x, point.y, point.z)
    # global middle
    # middle = point.x

    def spin_once(self):
        print("Reading")
        print(self.main_board.read())




if __name__ == '__main__':
    rospy.init_node('serial_communication', anonymous=True)
    rate = rospy.Rate(25)
    serial_communication = SerialCommunication()

    while not rospy.is_shutdown():
        serial_communication.spin_once()
        rate.sleep()
