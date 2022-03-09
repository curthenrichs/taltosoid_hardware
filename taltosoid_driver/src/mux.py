#! /usr/bin/env python3

import rospy

from std_msgs.msg import Uint32
from taltosoid_driver.msg import SolverTargets


class MuxNode:

    def __init__(self, num_channels, topicType, pubTopicName, subTopicName, ctrlTopicName):
        self.num_channels = num_channels
        self.channel = 0

        self._pub = rospy.Publisher(pubTopicName, topicType, queue_size=5)
        self._ctrl_sub = rospy.Subscriber(ctrlTopicName, Uint32, self._ctrl_cb)

        self._subs = []
        for i in range(0,self.num_channels):
            self._subs.append(rospy.Subscriber('{0}_channel_{1}'.format(subTopicName,i), topicType, lambda x: self._sub_cb(x, i)))

    def _sub_cb(self, msg, channel):
        if channel != self.channel:
            return # No pass through on non-active channel
        else:
            self._pub.publish(msg)

    def _ctrl_cb(self, msg):
        self.channel = msg.data


if __name__ == "__main__":
    rospy.init_node('mux')
    node = MuxNode(2, SolverTargets, '/taltosoid_driver/targets', 'targets_mux/input', 'targets_mux/control')
    rospy.spin()