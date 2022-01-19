#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is debug node.
subscribe No topcs.
Publish 'judge_info' topic. 
mainly use for simple sample program

by yoda-ocu.
'''

import rospy
import tf

class MoveDebugBot():
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()


    def strategy(self):
        base_time = rospy.Time.now()

        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                rospy.loginfo("trans:{} rot:{}".format(trans,rot))
            except:
                rospy.loginfo("Wait for tf")
                continue

if __name__ == '__main__':
    rospy.init_node('debug_node')
    bot = MoveDebugBot()
    bot.strategy()
