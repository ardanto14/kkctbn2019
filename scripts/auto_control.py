#!/usr/bin/env python
import rospy
from mavros_msgs.msg import RCIn
from kkctbn2019.msg import AutoControl

def rc_callback(msg):
    try:
        pwm = msg.channels[6];
    except IndexError:
        pwm = 1300

    control = AutoControl()
    if pwm < 1400:
        control.state = AutoControl.MANUAL;
    elif pwm > 1600:
        control.state = AutoControl.AVOID_RED_AND_GREEN;
    else:
        control.state = AutoControl.AVOID_RED;

    auto_control_publisher.publish(control)

if __name__  == '__main__':
    rospy.init_node("auto_control")

    rc_subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
    auto_control_publisher = rospy.Publisher("/makarax/auto_control", AutoControl, queue_size=8)
    
    rospy.spin()
