#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64
from kkctbn2019.msg import Config, ObjectCount, AutoControl

data = None
MIN_AREA = 600

config = Config()

config.red_l_h = 110
config.red_l_s = 56
config.red_l_v = 0
config.red_u_h = 255
config.red_u_s = 255
config.red_u_v = 255

config.green_l_h = 69
config.green_l_s = 43
config.green_l_v = 0
config.green_u_h = 99
config.green_u_s = 255
config.green_u_v = 255

config.brightness = 0
config.contrast = 0
config.gamma = 1

auto_control = AutoControl()
auto_control.state = AutoControl.AVOID_RED_AND_GREEN

def nothing(x):
    pass

def config_callback(config_in):
    global config
    config = config_in

def auto_control_callback(msg):
    auto_control = msg.state

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
    auto_control_subscriber = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)

    object_count_publisher = rospy.Publisher('/makarax/object/count', ObjectCount, queue_size=8)
    state_publisher = rospy.Publisher("state", Float64, queue_size=8)
    proccessed_image_publisher = rospy.Publisher("/makarax/image/proccessed/compressed", CompressedImage, queue_size=8)
    red_mask_publisher = rospy.Publisher("/makarax/image/mask/red/compressed", CompressedImage, queue_size=8)
    green_mask_publisher = rospy.Publisher("/makarax/image/mask/green/compressed", CompressedImage, queue_size=8)
    
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
        kernel = np.ones((5, 5), np.uint8)
        hsv = cv2.erode(hsv, kernel)

        # set ROI
        roi_y = config.roi_y
        cv2.line(frame, (width, roi_y), (0, roi_y), (0,255,0), 2)
        
        # RED
        red_l_h = config.red_l_h
        red_l_s = config.red_l_s
        red_l_v = config.red_l_v
        red_u_h = config.red_u_h
        red_u_s = config.red_u_s
        red_u_v = config.red_u_v

        lower_red = np.array([red_l_h, red_l_s, red_l_v])
        upper_red = np.array([red_u_h, red_u_s, red_u_v])

        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        # red_mask = cv2.Canny(hsv, 50, 100)
        # kernel = np.ones((5, 5), np.uint8)

        # red_mask = cv2.dilate(red_mask, kernel)
        # red_mask = cv2.erode(red_mask, kernel)
        
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
                    cv2.putText(frame, "Circle Red", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
                    count_red += 1
                    if x < min_x:
                        min_x = x

        
        # GREEN
        green_l_h = config.green_l_h
        green_l_s = config.green_l_s
        green_l_v = config.green_l_v
        green_u_h = config.green_u_h
        green_u_s = config.green_u_s
        green_u_v = config.green_u_v

        lower_green = np.array([green_l_h, green_l_s, green_l_v])
        upper_green = np.array([green_u_h, green_u_s, green_u_v])

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        # green_mask = cv2.Canny(hsv, 50, 100)
        # kernel = np.ones((5, 5), np.uint8)

        # green_mask = cv2.dilate(green_mask, kernel)
        # green_mask = cv2.erode(green_mask, kernel)

        # Contours detection
        max_x = 0
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
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
                    cv2.putText(frame, "Circle Green", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
                    count_green += 1
                    if x > max_x:
                        max_x = x

        
        # cv2.imshow("Frame", frame)
        # cv2.imshow("red_mask", red_mask)
        # cv2.waitKey(30)
        objectCount = ObjectCount()
        objectCount.red = count_red
        objectCount.green = count_green
        object_count_publisher.publish(objectCount)
        
        state = Float64()
        state.data = min_x
        state_publisher.publish(state)

        if auto_control.state == AutoControl.AVOID_RED_AND_GREEN and count_green > 0:
            state = Float64()
            state.data = min_x - 640
            state_publisher.publish(state)

        published_red_mask = CompressedImage()
        published_red_mask.header.stamp = rospy.Time.now()
        published_red_mask.format = "jpeg"
        published_red_mask.data = np.array(cv2.imencode(".jpg", red_mask)[1]).tostring()
        red_mask_publisher.publish(published_red_mask)

        published_green_mask = CompressedImage()
        published_green_mask.header.stamp = rospy.Time.now()
        published_green_mask.format = "jpeg"
        published_green_mask.data = np.array(cv2.imencode(".jpg", green_mask)[1]).tostring()
        green_mask_publisher.publish(published_green_mask)

        proccessed_image = CompressedImage()
        proccessed_image.header.stamp = rospy.Time.now()
        proccessed_image.format = "jpeg"
        proccessed_image.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
        proccessed_image_publisher.publish(proccessed_image)
