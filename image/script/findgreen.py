#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2

class Image_Process():

    def LocateTheBallCenter(self,frame):

        lower_green = np.array([36,0,0])
        upper_green = np.array([86,255,255])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_green, upper_green)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        #res = cv2.bitwise_and(frame, frame, mask = mask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        Center = None
        Lenq = 10

        if len(cnts) > 0:

            # print(len(cnts))

            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if radius > 7:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print(center)

    def HowFarToTheBall(depth_frame,coordinate,size):




