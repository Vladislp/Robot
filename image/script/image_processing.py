#!/usr/bin/env python

import numpy as np
import cv2
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from collections import deque

class ImageProcessing():

    def __init__(self):

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.hsv = None
        self.lower_green = None
        self.upper_green = None
        self.debug_img_pub = rospy.Publisher("image_processing/compressed", CompressedImage, queue_size=1)
        self.basket_publisher = rospy.Publisher("basket_coordinates", Point, queue_size=10)
        self.pub = rospy.Publisher("ball_coordinates", Point, queue_size=10)

    def LocateBasket(self):

        lower_blue = np.array([101, 247, 104])
        upper_blue = np.array([125, 255, 192])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        #mask = cv2.erode(mask, None, iterations=1)
        #mask = cv2.dilate(mask, None, iterations=1)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center1 = None
        if len(cnts) > 0:

            c = max(cnts, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            # only proceed if the basket meets a minimum size
            if w * h > 8:
                center1 = (int(x), int(y))
                print("I see blue basket")
                print(center1)
        return center1

    def LocateMagentaBasket(self):

        lower_magenta = np.array([165, 150, 50])
        upper_magenta = np.array([210, 255, 200])

        mask = cv2.inRange(hsv, lower_magenta, upper_magenta)
        #mask = cv2.erode(mask, None, iterations=1)
        #mask = cv2.dilate(mask, None, iterations=1)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center1 = None
        if len(cnts) > 0:

            c = max(cnts, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            # only proceed if the basket meets a minimum size
            if w * h > 8:
                center1 = (int(x), int(y))
                print("I see magenta basket")
                print(center1)
        return center1

    def spin_once(self):

        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        frame = aligned_frames.get_color_frame()

        depth_image_np = np.asanyarray(aligned_depth_frame.get_data())
        frame_np = np.asanyarray(frame.get_data())

        global hsv
        # Our operations on the frame come here
        hsv = cv2.cvtColor(frame_np, cv2.COLOR_BGR2HSV)

        lower_green = np.array([25, 110, 56])
        upper_green = np.array([90, 255, 146])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        res = cv2.bitwise_and(frame_np, frame_np, mask=mask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]


        center = None
        lenq = 10  # Maximum number of center points stored in memory

        if len(cnts) > 0:
            # print(len(cnts))

            c = max(cnts, key=cv2.contourArea)

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if radius > 3:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #print("I see ball")
                #print(center)
                self.pub.publish(Point(center[0],center[1],0))
        else:
            self.pub.publish(Point(-1, -1, -1))

        # cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # TO get center pixel values of image
        #print(hsv[320][240])

        # Display the resulting frame
        #cv2.imshow("frame", res)
        #cv2.waitKey(1)
        #test

        self.publish_img(mask)



    def publish_img(self, image):

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        result, encimg = cv2.imencode('.jpg', image, encode_param)
        msg.data = np.array(encimg).tostring()

        self.debug_img_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("image_processing", anonymous=True)
    process = ImageProcessing()
    config = rs.config()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        process.spin_once()
        process.LocateBasket()
        process.LocateMagentaBasket()
        rate.sleep()

    cv2.destroyAllWindows()
