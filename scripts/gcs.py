#!/usr/bin/env python
import cv2
import rospy
from kkctbn2019.msg import Config
import Tkinter
from PIL import Image, ImageTk
from sensor_msgs.msg import CompressedImage
import numpy as np

ori = np.zeros([480,640,3], dtype=np.uint8)
mask = np.zeros([480,640,3], dtype=np.uint8)

def add_slider(text, from_, to_, resolution, default=0):
    frame = Tkinter.Frame(master=master)
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
    global mask
    mask_cv2 = np.fromstring(img.data, np.uint8)
    mask_cv2 = cv2.imdecode(mask_cv2, cv2.IMREAD_COLOR)
    mask = mask_cv2


if __name__ == '__main__':
    rospy.init_node('gcs', anonymous=True)
    config_publisher = rospy.Publisher("/makarax/config", Config, queue_size=8)
    image_subscriber = rospy.Subscriber("/makarax/image/proccessed/compressed", CompressedImage, image_callback)
    red_mask_subscriber = rospy.Subscriber("/makarax/image/mask/red/compressed", CompressedImage, red_mask_callback)

    master = Tkinter.Tk()
    master.title("Config")

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

    img_frame = Tkinter.Frame(master=master, width=1280, height=960)

    ori_label = Tkinter.Label(img_frame, image=None)
    ori_label.pack(side="left")

    mask_label = Tkinter.Label(img_frame, image=None)
    mask_label.pack(side="left")

    img_frame.pack()

    while not rospy.is_shutdown():
        global ori
        if ori is not None:
            b,g,r = cv2.split(ori)
            img = cv2.merge((r,g,b)) 
            im = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=im)
            ori_label.config(image=imgtk)

        global mask
        if mask is not None:
            b,g,r = cv2.split(mask)
            img = cv2.merge((r,g,b)) 
            mask_im = Image.fromarray(img)
            mask_tk = ImageTk.PhotoImage(image=mask_im)
            mask_label.config(image=mask_tk)

        master.update()

        msg = Config()
        msg.l_h = red_l_h.get()
        msg.l_s = red_l_s.get()
        msg.l_v = red_l_v.get()
        msg.u_h = red_u_h.get()
        msg.u_s = red_u_s.get()
        msg.u_v = red_u_v.get()
        msg.brightness = brightness.get()
        msg.contrast = contrast.get()
        msg.gamma = gamma.get()
        msg.roi_y = roi_y.get()

        config_publisher.publish(msg)