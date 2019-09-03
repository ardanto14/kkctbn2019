#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

if __name__  == '__main__':
    rospy.init_node("receiver")
    image_publisher = rospy.Publisher("/makarax/image", Image, queue_size=8)
    compressed_image_publisher = rospy.Publisher("/makarax/image/compressed", CompressedImage, queue_size=8)
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        _, data = cap.read()
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode(".jpg", data)[1]).tostring()
        imgmsg = bridge.cv2_to_imgmsg(data, "bgr8")
        
        image_publisher.publish(imgmsg)
        compressed_image_publisher.publish(msg)