#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
# First import the library
import pyrealsense2 as rs
# Import OpenCV for easy image rendering
import cv2
# Import Numpy for easy array manipulation
import numpy as np
from collections import deque

lower_bound = np.array([60, 100, 40])
upper_bound = np.array([90, 255, 255])


class ImageProcessing():
    def __init__(self):
        self.pipeline = None
        self.align = None
        self.depth_image = None
        self.regular_image = None
        self.yuv = None
        self.hsv = None
        self.pub = rospy.Publisher('ball_coordinates', Point, queue_size=10)


    def run(self):#image
        #Create a pipeline
        self.pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        # different resolutions of color and depth streams
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)

        # Start streaming
        self.pipeline.start(config)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame(self):
        # main_board.launch_motor(-10, 10, 0, 0)
        # rospy.logdebug('dededaij')
        # Create a pipeline
        # Get frameset of color and depth

        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are
        if not aligned_depth_frame or not color_frame:
            #rospy.loginfo("Failed to load frames!")
            return

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.regular_image = np.asanyarray(color_frame.get_data())
        # self.yuv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2YUV)
        self.hsv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2HSV)

    def detect_ball(self):
        mask = cv2.inRange(self.hsv, lower_bound, upper_bound)
        #rospy.loginfo("{} {}".format(len(self.hsv[0]), len(mask[0])))

        im2, contours, hierarchie = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        biggest_ball_size = 0
        biggest_ball_rect = None
        #rospy.loginfo("Found {} contours".format(len(contours)))

        for contour in contours:
            #rospy.loginfo("found a contour")
            contour_size = cv2.contourArea(contour)
            if contour_size > biggest_ball_size:
                biggest_ball_rect = cv2.boundingRect(contour)
                biggest_ball_size = contour_size

        if biggest_ball_rect is not None:

            x, y, w, h = biggest_ball_rect
            center_x = (x + w) / 2
            center_y = (y + h) / 2
            #rospy.loginfo("Found the biggest ball with bounding rect: {}".format(biggest_ball_rect))
            #rospy.loginfo("Ball: {} {}".format(center_x, center_y))
            self.pub.publish(Point(center_x, center_y, 0))

            #self.print_mask(mask)

    def print_mask(self, mask):
        coverage = [0] * 64
        for y in range(480):
            for x in range(640):
                c = mask[y, x]
                if c > 128:
                    coverage[x // 10] += 1

            if y % 20 is 19:
                line = ""
                for c in coverage:
                    line += " .:nhBXWW"[c // 25]
                coverage = [0] * 64
                rospy.loginfo(line)


# NOT UNDER THE CLASS
if __name__ == '__main__':
    rospy.init_node('image_processing', anonymous=True)
    rate = rospy.Rate(20)  # 60hz
    cam_proc = ImageProcessing()
    cam_proc.run()

    while not rospy.is_shutdown():
        cam_proc.get_frame()
        cam_proc.detect_ball()
rate.sleep()