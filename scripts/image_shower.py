#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

def callback1(image):
    print("kesana")
    bridge = CvBridge()
    ori_cv2 = bridge.imgmsg_to_cv2(image)
    # red_mask_cv2 = bridge.imgsmsg_to_cv2(red_mask)
    # green_mask_cv2 = bridge.imgsmsg_to_cv2(green_mask)
    cv2.imshow("ori", ori_cv2)
    # imshow("Red Mask", red_mask_cv2)
    # imshow("Green Mask", green_mask_cv2)
    cv2.waitKey(30)

if __name__  == '__main__':
    rospy.init_node('image_shower', anonymous=True)
    image_subscriber = rospy.Subscriber('/makarax/image', Image, callback1)
    
    # red_mask_subscribet = rospy.Subscriber('/makarax/image', Image, callback1)
    # green_mask_subscriber = rospy.Subscriber('/makarax/image', Image, callback1)
    cv2.namedWindow("ori")
    cv2.startWindowThread()
    # cv2.namedWindow("Red Mask")
    # cv2.namedWindow("Green Mask")
    # cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        # print("kesini")
        ori = rospy.wait_for_message('/makarax/image', Image)
        # _, ori = cap.read()
        # red_mask = rospy.wait_for_message('/makarax/image/mask/red', Image)
        # green_mask = rospy.wait_for_message('/makarax/image/mask/green', Image)
        
        rospy.spin()