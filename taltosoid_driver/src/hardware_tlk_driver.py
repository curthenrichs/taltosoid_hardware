#!/usr/bin/env python

'''
TODO Documentation
'''

import json
import rospy

from core import JOINT_LIST, EE_LINK, BASE_LINK
from sensor_msgs.msg import JointState


class HardwareDriver:

    def __init__(self):
        self._js_pub = rospy.Publisher('/taltosoid_driver/hardware/joint_state', JointState, queue_size=5)

    def spin(self):
        pass


if __name__ == "__main__":
    rospy.init_node('taltosoid_driver')
    node = HardwareDriver()
    node.spin()
