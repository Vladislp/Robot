 #!/usr/bin/env python
 # license removed for brevity
import rospy
from std_msgs.msg import String

import pyrealsense2 as rs
import numpy as np
import cv2
from locateBall import LocateBallCenter

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


def SendBallLocation():

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "%s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
   try:
       SendBallLocation()
   except rospy.ROSInterruptException:
pass