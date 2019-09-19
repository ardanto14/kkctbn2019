#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt16, Float64
from kkctbn2019.msg import Config

font = cv2.FONT_HERSHEY_COMPLEX
data = None
MIN_AREA = 1000

global config
config = Config()
config.l_h = 0
config.l_s = 0
config.l_v = 0
config.u_h = 255
config.u_s = 255
config.u_v = 255
config.brightness = 50
config.contrast = 50
config.gamma = 0.1

def nothing(x):
    pass

def config_callback(config_in):
    global config
    config = config_in

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)

if __name__ == '__main__':
    rospy.init_node('image_processing', anonymous=True)
    global config
    image_subscriber = rospy.Subscriber('/makarax/image', Image, nothing)
    config_subscriber = rospy.Subscriber("/makarax/config", Config, config_callback)
    publisher_red = rospy.Publisher('/makarax/object/count/red', UInt16, queue_size=8)
    # publisher_green = rospy.Publisher('/makarax/object/count/green', UInt16, queue_size=8)
    state_publisher = rospy.Publisher("state", Float64, queue_size=8)
    proccessed_image_publisher = rospy.Publisher("/makarax/image/proccessed/compressed", CompressedImage, queue_size=8)
    red_mask_publisher = rospy.Publisher("/makarax/image/mask/red/compressed", CompressedImage, queue_size=8)
    # green_mask_publisher = rospy.Publisher("/makarax/image/mask/green", Image, queue_size=8)
    
    while not rospy.is_shutdown():
        data = rospy.wait_for_message('/makarax/image', Image)
        # do stuff
        count_red = 0
        count_green = 0
        bridge = CvBridge()
        ori = bridge.imgmsg_to_cv2(data)

        brightness = config.brightness
        contrast = config.contrast
        gamma = config.gamma

        frame = ori.copy()

        if gamma == 0:
            pass
        else:
            frame = adjust_gamma(frame, gamma=gamma)

        # change contrast and brightness
        frame = np.int16(frame)
        frame = frame * (contrast/127+1) - contrast + brightness
        frame = np.clip(frame, 0, 255)
        frame = np.uint8(frame)

        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 1)

        # set ROI
        roi_y = config.roi_y
        cv2.line(frame, (width, roi_y), (0, roi_y), (0,255,0), 2)
        
        # RED
        l_h = config.l_h
        l_s = config.l_s
        l_v = config.l_v
        u_h = config.u_h
        u_s = config.u_s
        u_v = config.u_v

        lower_red = np.array([l_h, l_s, l_v])
        upper_red = np.array([u_h, u_s, u_v])

        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        # red_mask = cv2.Canny(hsv, 50, 100)
        kernel = np.ones((5, 5), np.uint8)

        red_mask = cv2.dilate(red_mask, kernel)
        red_mask = cv2.erode(red_mask, kernel)
        
        # Contours detection
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        min_x = 9999
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])

            if area > MIN_AREA and y > roi_y:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                if 7 <= len(approx) < 20:
                    cv2.putText(frame, "Circle Red", (x, y), font, 0.5, (0, 0, 255))
                    count_red += 1
                    if x < min_x:
                        min_x = x

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
        # cv2.imshow("Frame", frame)
        # cv2.imshow("red_mask", red_mask)
        # cv2.waitKey(30)
        red = UInt16()
        red.data = count_red
        publisher_red.publish(red)
        
        state = Float64()
        state.data = min_x
        state_publisher.publish(state)
        # green = UInt16()
        # red.data = count_green
        # publisher_green.publish(green)

        published_red_mask = CompressedImage()
        published_red_mask.header.stamp = rospy.Time.now()
        published_red_mask.format = "jpeg"
        published_red_mask.data = np.array(cv2.imencode(".jpg", red_mask)[1]).tostring()
        red_mask_publisher.publish(published_red_mask)

        proccessed_image = CompressedImage()
        proccessed_image.header.stamp = rospy.Time.now()
        proccessed_image.format = "jpeg"
        proccessed_image.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
        proccessed_image_publisher.publish(proccessed_image)
