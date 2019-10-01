#!/usr/bin/env python
import rospy
from mavros_msgs.msg import RCIn
from kkctbn2019.msg import AutoControl

def rc_callback(msg):
    pwm = msg.channels[6];
    control = AutoControl()
    if pwm < 1400:
        control.state = AutoControl.AVOID_RED;
    else:
        control.state = AutoControl.AVOID_RED_AND_GREEN;

    auto_control_publisher.publish(control)

if __name__  == '__main__':
    rospy.init_node("auto_control")

    rc_subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
    auto_control_publisher = rospy.Publisher("/makarax/auto_control", AutoControl, queue_size=8)
    
    rospy.spin()
