#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from image_processing import image_processing
import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
img_handler = image_processing()

def talker():

    pub = rospy.Publisher('ball_coordinates', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    i = 0
    for i in range(60):
        print('Shutdown frames', i)

        frames = pipeline.wait_for_frames()
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

        color_image = np.asanyarray(color_frame.get_data())
        coordinates = image.processing.LocateBallCenter(color_image)
        if coordinates is None:
            coordinates = (-1, -1)

        hello_str = "{},,{}".format(coordinates[0], coordinates[1])
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        i += 1