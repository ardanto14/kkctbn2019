#!/usr/bin/env python
import cv2
import rospy
from kkctbn2019.msg import Config
import Tkinter

def add_slider(text, from_, to_, resolution, default=0):
    frame = Tkinter.Frame()
    label = Tkinter.Label(frame, text=text, fg='white', font=("Helvetica", 12))
    label.grid(row=1, column=1, padx=10, pady=10)
    scale = Tkinter.Scale(frame, from_=from_, to=to_, resolution=resolution, orient=Tkinter.HORIZONTAL, length=300)
    scale.set(default)
    scale.grid(row=1, column=2, padx=10, pady=0)
    frame.pack()
    return scale


if __name__ == '__main__':
    try:
        rospy.init_node('image_processing', anonymous=True)
        config_publisher = rospy.Publisher("/makarax/config", Config, queue_size=8)

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

        while not rospy.is_shutdown():
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

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()