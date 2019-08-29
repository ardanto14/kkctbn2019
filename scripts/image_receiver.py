#!/user/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

if __name__  == '__main__':
    rospy.init_node("receiver")
    image_publisher = rospy.Publisher("/makarax/image", Image, 8)
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        _, data = cap.read()
        imgmsg = bridge.cv2_to_imgmsg(data, "bgr8")
        image_publisher.publish(imgmsg)