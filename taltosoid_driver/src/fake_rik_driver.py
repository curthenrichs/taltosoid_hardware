#!/usr/bin/env python

'''
TODO Documentation
'''

import rospy

from core import JOINT_LIST
from sensor_msgs.msg import JointState
from relaxed_ik.msg import JointAngles


class FakeDriver:

    def __init__(self):
        self._js_pub = rospy.Publisher('/taltosoid_driver/joint_state', JointState, queue_size=5)
        self._rik_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, self._rik_update_cb)
        self._js_sub = rospy.Subscriber('/taltosoid_driver/joints_in', JointState, self._js_update_cb)

    def _rik_update_cb(self, msg):
        js = JointState()
        js.header = msg.header
        js.name = JOINT_LIST
        js.position = msg.angles.data
        js.velocity = [0] * len(js.position)
        js.effort = [0] * len(js.position)
        self._js_pub.publish(js)

    def _js_update_cb(self, msg):
        for j in msg.name:
            if not j in JOINT_LIST:
                return # ignore malformed messages
        self._js_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('taltosoid_driver')
    node = FakeDriver()
    rospy.spin()
