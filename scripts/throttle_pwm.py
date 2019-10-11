#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
import dynamic_reconfigure.client

current_pwm_int = 1750

def callback(config):
    global current_pwm_int
    current_pwm_int = config.pwm

    current_pwm = UInt16()
    current_pwm.data = current_pwm_int
    
    pwm_publisher.publish(current_pwm)

if __name__ == '__main__':
    rospy.init_node("mode")
    client = dynamic_reconfigure.client.Client("server", config_callback=callback)
    pwm_publisher = rospy.Publisher('/makarax/pwm_throttle', UInt16, queue_size=8)

    rospy.spin()



