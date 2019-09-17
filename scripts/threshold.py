#!/usr/bin/env python
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16, Float64
import random
from kkctbn2019.msg import Threshold

font = cv2.FONT_HERSHEY_COMPLEX
data = None
MIN_AREA = 1000

def nothing(x):
    pass

def callback1(image):
    pass  


if __name__ == '__main__':
    try:
        rospy.init_node('image_processing', anonymous=True)
        threshold_publisher = rospy.Publisher("/makarax/threshold", Threshold, queue_size=8)

        cv2.startWindowThread()
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("RED L-H", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("RED L-S", "Trackbars", 56, 255, nothing)
        cv2.createTrackbar("RED L-V", "Trackbars", 112, 255, nothing)
        cv2.createTrackbar("RED U-H", "Trackbars", 19, 255, nothing)
        cv2.createTrackbar("RED U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("RED U-V", "Trackbars", 255, 255, nothing)

        # cv2.createTrackbar("GREEN L-H", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("GREEN L-S", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("GREEN L-V", "Trackbars", 0, 255, nothing)
        # cv2.createTrackbar("GREEN U-H", "Trackbars", 255, 255, nothing)
        # cv2.createTrackbar("GREEN U-S", "Trackbars", 255, 255, nothing)
        # cv2.createTrackbar("GREEN U-V", "Trackbars", 255, 255, nothing)
        while not rospy.is_shutdown():
            l_h = cv2.getTrackbarPos("RED L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("RED L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("RED L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("RED U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("RED U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("RED U-V", "Trackbars")

            msg = Threshold()
            msg.l_h = l_h
            msg.l_s = l_s
            msg.l_v = l_v
            msg.u_h = u_h
            msg.u_s = u_s
            msg.u_v = u_v

            threshold_publisher.publish(msg)

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()