#!/usr/bin/env python

'''
TODO Documentation
'''

import rospy
from sensor_msgs.msg import JointState
from relaxed_ik.msg import JointAngles


JOINT_LIST = [
    'finger_root_to_rotation_root',
    'rotation_root_to_finger_segment_1',
    'finger_segment_1_to_finger_hinge_1',
    'finger_segment_2_to_finger_hinge_2'
]


class FakeDriver:

    def __init__(self):
        self._js_pub = rospy.Publisher('/taltosoid_driver/joint_state', JointState, queue_size=5)
        self._js_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, self._joint_update_cb)

    def _joint_update_cb(self, msg):
        js = JointState()
        js.header = msg.header
        js.name = JOINT_LIST
        js.position = msg.angles.data
        js.velocity = [0] * len(js.position)
        js.effort = [0] * len(js.position)
        self._js_pub.publish(js)


if __name__ == "__main__":
    rospy.init_node('taltosoid_driver')

    node = FakeDriver()

    rospy.spin()
