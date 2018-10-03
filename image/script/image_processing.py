#!/usr/bin/env python

import numpy as np
import cv2
import pyrealsense2 as rs
import rospy


class ImageProcessing():

    def __init__(self):

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

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

        # Our operations on the frame come here
        hsv = cv2.cvtColor(frame_np, cv2.COLOR_BGR2HSV)

        lower_green = np.array([50, 120, 60])
        upper_green = np.array([70, 175, 100])

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
            if radius > 7:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print(center)

        # cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # TO get center pixel values of image
        # print(hsv[320][240])

        # Display the resulting frame
        cv2.imshow("frame", res)
        cv2.waitKey(1)

    def detect_ball(self):
        mask = cv2.inRange(self.hsv, lower_green, upper_green)
        # rospy.loginfo("{} {}".format(len(self.hsv[0]), len(mask[0])))

        im2, contours, hierarchie = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        biggest_ball_size = 0
        biggest_ball_rect = None
        # rospy.loginfo("Found {} contours".format(len(contours)))

        for contour in contours:
            # rospy.loginfo("found a contour")
            contour_size = cv2.contourArea(contour)
            if contour_size > biggest_ball_size:
                biggest_ball_rect = cv2.boundingRect(contour)
                biggest_ball_size = contour_size

        if biggest_ball_rect is not None:
            x, y, w, h = biggest_ball_rect
            center_x = (x + w) / 2
            center_y = (y + h) / 2
            rospy.loginfo("Found the biggest ball with bounding rect: {}".format(biggest_ball_rect))
            # rospy.loginfo("Ball: {} {}".format(center_x, center_y))
            #self.pub.publish(Point(center_x, center_y, 0))



if __name__ == '__main__':
    rospy.init_node("image_processing", anonymous=True)
    process = ImageProcessing()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        process.spin_once()
        rate.sleep()

    cv2.destroyAllWindows()
