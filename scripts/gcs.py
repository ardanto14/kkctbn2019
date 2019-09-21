#!/usr/bin/env python
import cv2
import rospy
from kkctbn2019.msg import Config
import Tkinter
from PIL import Image, ImageTk
from sensor_msgs.msg import CompressedImage
import numpy as np
from std_msgs.msg import UInt16
from kkctbn2019.msg import Mode

ori = np.zeros([480,640,3], dtype=np.uint8)
red_mask = np.zeros([480,640,3], dtype=np.uint8)
green_mask = np.zeros([480,640,3], dtype=np.uint8)
throttlePwm = 0
mode = "NONE"

def add_slider(text, from_, to_, resolution, default=0):
    frame = Tkinter.Frame(master=slider_frame)
    label = Tkinter.Label(frame, text=text, fg='black', font=("Helvetica", 12))
    label.grid(row=1, column=1, padx=10, pady=10)
    scale = Tkinter.Scale(frame, from_=from_, to=to_, resolution=resolution, orient=Tkinter.HORIZONTAL, length=300)
    scale.set(default)
    scale.grid(row=1, column=2, padx=10, pady=0)
    frame.pack()
    return scale

def image_callback(img):
    global ori
    ori_cv2 = np.fromstring(img.data, np.uint8)
    ori_cv2 = cv2.imdecode(ori_cv2, cv2.IMREAD_COLOR)
    ori = ori_cv2

def red_mask_callback(img):
    global red_mask
    mask_cv2 = np.fromstring(img.data, np.uint8)
    mask_cv2 = cv2.imdecode(mask_cv2, cv2.IMREAD_COLOR)
    red_mask = mask_cv2

def green_mask_callback(img):
    global green_mask
    mask_cv2 = np.fromstring(img.data, np.uint8)
    mask_cv2 = cv2.imdecode(mask_cv2, cv2.IMREAD_COLOR)
    green_mask = mask_cv2 

def throttle_pwm_callback(pwm):
    global throttlePwm
    throttlePwm = pwm.data

def mode_callback(mode_in):
    global mode
    if mode_in.value == Mode.MANUAL:
        mode = "MANUAL"
    elif mode_in.value == Mode.AUTO:
        mode = "AUTO"
    elif mode_in.value == Mode.HOLD:
        mode = "HOLD"

if __name__ == '__main__':
    rospy.init_node('gcs', anonymous=True)
    config_publisher = rospy.Publisher("/makarax/config", Config, queue_size=8)
    image_subscriber = rospy.Subscriber("/makarax/image/proccessed/compressed", CompressedImage, image_callback)
    red_mask_subscriber = rospy.Subscriber("/makarax/image/mask/red/compressed", CompressedImage, red_mask_callback)
    green_mask_subscriber = rospy.Subscriber("/makarax/image/mask/green/compressed", CompressedImage, green_mask_callback)
    throttle_pwm_subscriber = rospy.Subscriber("/makarax/pwm/throttle", UInt16, throttle_pwm_callback)
    mode_subscriber = rospy.Subscriber("/makarax/mode", Mode, mode_callback)

    master = Tkinter.Tk()
    master.title("Config")

    slider_frame = Tkinter.Frame(master=master)

    contrast = add_slider('Contrast', -255, 255, 1)
    brightness = add_slider('Brightness', -127, 127, 1)
    gamma = add_slider('Gamma', 0.1, 3, 0.1, 1)
    roi_y = add_slider('ROI Y', 0, 480, 1)
    red_l_h = add_slider('RED L-H', 0, 255, 1, 0)
    red_l_s = add_slider('RED L-S', 0, 255, 1, 56)
    red_l_v = add_slider('RED L-V', 0, 255, 1, 112)
    red_u_h = add_slider('RED U-H', 0, 255, 1, 19)
    red_u_s = add_slider('RED U-S', 0, 255, 1, 255)
    red_u_v = add_slider('RED U-V', 0, 255, 1, 255)

    green_l_h = add_slider('GREEN L-H', 0, 255, 1, 0)
    green_l_s = add_slider('GREEN L-S', 0, 255, 1, 56)
    green_l_v = add_slider('GREEN L-V', 0, 255, 1, 112)
    green_u_h = add_slider('GREEN U-H', 0, 255, 1, 19)
    green_u_s = add_slider('GREEN U-S', 0, 255, 1, 255)
    green_u_v = add_slider('GREEN U-V', 0, 255, 1, 255)

    slider_frame.grid(row=1, column=1)

    info_frame = Tkinter.Frame(master=master)
    pwmLabel = Tkinter.Label(info_frame, text="PWM Throttle: " + str(throttlePwm), fg='black', font=("Helvetica", 12))
    modeLabel = Tkinter.Label(info_frame, text="Mode: " + mode, fg='black', font=("Helvetica", 12))
    pwmLabel.pack()
    modeLabel.pack()

    info_frame.grid(row=1, column=2)

    ori_label = Tkinter.Label(master=master, image=None)
    ori_label.grid(row=2, column=1)

    red_mask_label = Tkinter.Label(master=master, image=None)
    red_mask_label.grid(row=2, column=2)

    green_mask_label = Tkinter.Label(master=master, image=None)
    green_mask_label.grid(row=2, column=3)

    while not rospy.is_shutdown():
        if ori is not None:
            b,g,r = cv2.split(ori)
            img = cv2.merge((r,g,b)) 
            im = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=im)
            ori_label.config(image=imgtk)

        if red_mask is not None:
            b,g,r = cv2.split(red_mask)
            img_red = cv2.merge((r,g,b)) 
            mask_im_red = Image.fromarray(img_red)
            mask_tk_red = ImageTk.PhotoImage(image=mask_im_red)
            red_mask_label.config(image=mask_tk_red)
        
        if green_mask is not None:
            b,g,r = cv2.split(green_mask)
            img_green = cv2.merge((r,g,b)) 
            mask_im_green = Image.fromarray(img_green)
            mask_tk_green = ImageTk.PhotoImage(image=mask_im_green)
            green_mask_label.config(image=mask_tk_green)

        pwmLabel.config(text="PWM Throttle: " + str(throttlePwm))
        modeLabel.config(text="Mode: " + mode)
        master.update()

        msg = Config()
        msg.red_l_h = red_l_h.get()
        msg.red_l_s = red_l_s.get()
        msg.red_l_v = red_l_v.get()
        msg.red_u_h = red_u_h.get()
        msg.red_u_s = red_u_s.get()
        msg.red_u_v = red_u_v.get()
        msg.green_l_h = green_l_h.get()
        msg.green_l_s = green_l_s.get()
        msg.green_l_v = green_l_v.get()
        msg.green_u_h = green_u_h.get()
        msg.green_u_s = green_u_s.get()
        msg.green_u_v = green_u_v.get()
        msg.brightness = brightness.get()
        msg.contrast = contrast.get()
        msg.gamma = gamma.get()
        msg.roi_y = roi_y.get()

        config_publisher.publish(msg)
