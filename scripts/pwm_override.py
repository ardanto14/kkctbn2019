#!/usr/bin/env python
import rospy
import threading 
from kkctbn2019.msg import AutoControl
from std_msgs.msg import Bool

auto_control_before = AutoControl()

def timer_action():
    msg = Bool()
    msg.data = True
    print("sini")
    pwm_override_publisher.publish(msg)

def auto_control_callback(msg):
    if msg.state == AutoControl.AVOID_RED_AND_GREEN and auto_control_before.state != AutoControl.AVOID_RED_AND_GREEN:
        published_data = Bool()
        published_data.data = False
        pwm_override_publisher.publish(published_data)
        
        timer = threading.Timer(9.0, timer_action)
        timer.start()
    elif msg.state != AutoControl.AVOID_RED_AND_GREEN and auto_control_before.state == AutoControl.AVOID_RED_AND_GREEN:
        published_data = Bool()
        published_data.data = False
        pwm_override_publisher.publish(published_data)
    
    global auto_control_before
    auto_control_before = msg

    

if __name__  == '__main__':
    rospy.init_node("pwm_override")

    pwm_override_publisher = rospy.Publisher("/makarax/pwm_override", Bool, queue_size=8)

    auto_control_publisher = rospy.Subscriber("/makarax/auto_control", AutoControl, auto_control_callback)
    
    rospy.spin()
