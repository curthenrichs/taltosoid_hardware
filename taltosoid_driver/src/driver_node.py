#!/usr/bin/env python

'''
TODO Documentation
'''

import rospy
from driver import Driver
import solver as LTK_Solver

from .constants import JOINT_LIST, EE_LINK, BASE_LINK

from sensor_msgs.msg import JointState
from taltosoid_driver.msg import SolverTargets, DriverError
from taltosoid_driver.srv import ChangePortService


class HardwareDriver:

    def __init__(self, default_port=None):
        self._js_pub = rospy.Publisher('/taltosoid_driver/joint_state', JointState, queue_size=5)
        self._err_pub = rospy.Publisher('./taltosoid_driver/error', DriverError, queue_size=5)

        self._target_sub = rospy.Subscriber('/taltosoid_driver/targets', SolverTargets, self._target_cb)

        self._change_port_srv = rospy.Service('/taltosoid_driver/change_port', ChangePortService, self._change_port_cb)

    def spin(self):
        pass

    def _target_cb(self, msg):
        pass

    def _change_port_cb(self, request):
        pass


if __name__ == "__main__":
    rospy.init_node('taltosoid_driver')

    default_port = rospy.get_param('~default_port',None)

    node = HardwareDriver(default_port)
    node.spin()
