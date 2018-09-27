import numpy as np
import cv2
import pyrealsense2 as rs
from collections import deque

class ImageHandler():

    def LocateBall(self,frame):

        greenIp = (90, 225, 255)
        greenLow = (60, 100, 40)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, greenIp, greenLow)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None
        lenq = 10

        pts = deque(maxlen=lenq)

        if len(cnts) > 0:

            c = max(cnts, key=cv2,contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)

            if radius > 7:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                return center

    def howfar(depth_frame,coordinate,size):
        '''Returns the distance  in mm around a center point a square of size s'''

        # Distance from the camera plane(not camera center) to the object in mm
        xc,yc = coordinate
        avg_dist = 0

        xmin = max(xc-size,0)
        xmax = min(xc+size,640)
        ymin = max(yc-size,0)
        ymax = min(yc+size,480)

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        for x in range(xmin, xmax):
            for y in range(ymin, ymax):
                depth = depth_frame.get_distance(x,y)
                avg_dist = + depth
                #depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [y, x], depth)
                # depth_point x y z
        avg_dist /= (xmax - xmin + ymax - ymin)
        # Distance respect to the camera
        cam_angle = 45
        cam_height = 200
        return dist