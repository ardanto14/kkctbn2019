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

        sub = rospy.Subscriber('/makarax/image', Image, callback1)
        publisher_red = rospy.Publisher('/makarax/object/count/red', UInt16, queue_size=8)
        # publisher_green = rospy.Publisher('/makarax/object/count/green', UInt16, queue_size=8)
        # setpoint_publisher = rospy.Publisher("setpoint", Float64, queue_size=1)
        state_publisher = rospy.Publisher("state", Float64, queue_size=1)
        red_mask_publisher = rospy.Publisher("/makarax/image/mask/red", Image, queue_size=8)
        green_mask_publisher = rospy.Publisher("/makarax/image/mask/green", Image, queue_size=8)

        bridge = CvBridge()

        cv2.startWindowThread()
        cv2.namedWindow("Trackbars")
        cv2.namedWindow("Frame")
        cv2.namedWindow("red_mask")
        # cv2.namedWindow("green_mask")
        cv2.createTrackbar("RED L-H", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("RED L-S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("RED L-V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("RED U-H", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("RED U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("RED U-V", "Trackbars", 255, 255, nothing)

        cv2.createTrackbar("GREEN L-H", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("GREEN L-S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("GREEN L-V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("GREEN U-H", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("GREEN U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("GREEN U-V", "Trackbars", 255, 255, nothing)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('/makarax/image', Image)
            # do stuff
            count_red = 0
            count_green = 0
            bridge = CvBridge()
            ori = bridge.imgmsg_to_cv2(data)
            frame = ori.copy()
            height, width = frame.shape[:2]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (5, 5), 1)
            cv2.line(frame, (width, height/2), (0, height/2), (0,255,0), 2)
            cv2.line(frame, (width/3, height), (width/3, 0), (0,255,0), 2)
            
            # RED
            l_h = cv2.getTrackbarPos("RED L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("RED L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("RED L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("RED U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("RED U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("RED U-V", "Trackbars")

            lower_red = np.array([l_h, l_s, l_v])
            upper_red = np.array([u_h, u_s, u_v])

            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            # red_mask = cv2.Canny(hsv, 50, 100)
            kernel = np.ones((5, 5), np.uint8)

            red_mask = cv2.dilate(red_mask, kernel)
            red_mask = cv2.erode(red_mask, kernel)
            published_red_mask = bridge.cv2_to_imgmsg(red_mask)
            red_mask_publisher.publish(published_red_mask)
            # Contours detection
            contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            max_area = 0
            max_contour = None
            max_x = 0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                
                if area > MIN_AREA and y > height/2:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)
                    cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                    if 7 <= len(approx) < 20:
                        cv2.putText(frame, "Circle Red", (x, y), font, 0.5, (0, 0, 255))
                        count_red += 1
                        if area > max_area:
                            max_area = area
                            max_contour = cnt
                            max_x = x

            '''
            # GREEN
            l_h = cv2.getTrackbarPos("GREEN L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("GREEN L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("GREEN L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("GREEN U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("GREEN U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("GREEN U-V", "Trackbars")

            lower_green = np.array([l_h, l_s, l_v])
            upper_green = np.array([u_h, u_s, u_v])

            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            # green_mask = cv2.Canny(hsv, 50, 100)
            kernel = np.ones((5, 5), np.uint8)

            green_mask = cv2.dilate(green_mask, kernel)
            green_mask = cv2.erode(green_mask, kernel)
            published_green_mask = bridge.cv2_to_imgmsg(green_mask)
            red_mask_publisher.publish(published_green_mask)

            # Contours detection
            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

            for cnt in contours:
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                moments = cv2.moments(cnt)
                x = int(moments['m10'] / moments['m00'])
                y = int(moments['m01'] / moments['m00'])
                if area > MIN_AREA:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                    if 7 <= len(approx) < 20:
                        cv2.putText(frame, "Circle Green", (x, y), font, 0.5, (0, 0, 255))
                        count_green += 1
            '''
            cv2.imshow("Frame", frame)
            cv2.imshow("red_mask", red_mask)
            cv2.waitKey(30)
            red = UInt16()
            red.data = count_red
            publisher_red.publish(red)
            print(max_x)
            state = Float64()
            if max_x == 0:
                continue
            state.data = max_x
            state_publisher.publish(state)
            # green = UInt16()
            # red.data = count_green
            # publisher_green.publish(green)
            rate.sleep()

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()