#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from kkctbn2019.cfg import Kkctbn2019Config

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("server", anonymous = False)

    srv = Server(Kkctbn2019Config, callback)
    rospy.spin()