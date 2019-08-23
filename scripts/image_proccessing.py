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
from std_msgs.msg import UInt16
import random

font = cv2.FONT_HERSHEY_COMPLEX
data = None

def nothing(x):
    pass

def callback1(image):
    pass  


if __name__ == '__main__':
    try:
        rospy.init_node('image_processing', anonymous=True)
        sub = rospy.Subscriber('/makarax/image', Image, callback1)
        publisher = rospy.Publisher('/makarax/object/count', UInt16, queue_size=8)
        cv2.startWindowThread()
        cv2.namedWindow("Trackbars")
        cv2.namedWindow("Frame")
        cv2.namedWindow("Mask")
        cv2.createTrackbar("L-H", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L-S", "Trackbars", 66, 255, nothing)
        cv2.createTrackbar("L-V", "Trackbars", 126, 255, nothing)
        cv2.createTrackbar("U-H", "Trackbars", 180, 255, nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U-V", "Trackbars", 223, 255, nothing)
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('/makarax/image', Image)
            # do stuff
            count_a = 0
            count_b = 0
            count_c = 0
            count_d = 0
            bridge = CvBridge()
            ori = bridge.imgmsg_to_cv2(data)
            frame = ori.copy()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (5, 5), 1)

            l_h = cv2.getTrackbarPos("L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("U-V", "Trackbars")

            lower_red = np.array([l_h, l_s, l_v])
            upper_red = np.array([u_h, u_s, u_v])

            mask = cv2.inRange(hsv, lower_red, upper_red)
            # mask = cv2.Canny(hsv, 50, 100)
            kernel = np.ones((5, 5), np.uint8)

            # mask = cv2.dilate(mask, kernel)
            mask = cv2.erode(mask, kernel)

            # Contours detection
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

            for cnt in contours:
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]

                if area > 400:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                    if 7 <= len(approx) < 20:
                        cv2.putText(frame, "Circle", (x, y), font, 0.5, (0, 0, 255))
                        count_d += 1

            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)
            cv2.waitKey(30)
            count = UInt16()
            count.data = count_d
            publisher.publish(count)

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()