#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is calc_nearest_target node.
Subscribe tf.
Publish 'nearest_target' topic. 
mainly use for simple sample program

by yoda-ocu.
'''

import rospy

from std_msgs.msg import String
from burger_war_dev.msg import war_state
import tf

from local_map import *

class CalcNearestTargetBot():
    def __init__(self):
        self.rate = rospy.Rate(1)
        self.tf_listener = tf.TransformListener()
        self.sub = rospy.Subscriber("war_state_info", war_state, self.warStateCallback)
        self.pub = rospy.Publisher("nearest_target",String, queue_size=10)

    def strategy(self):
        base_time = rospy.Time.now()
        self.nearest_target = None
        self.war_state = None
        while not rospy.is_shutdown():
            if self.war_state is None:
                self.rate.sleep()
                continue
            
            try:
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                rospy.logdebug("trans:{} rot:{}".format(trans,rot))
            except:
                rospy.logwarn("Wait for tf")
                self.rate.sleep()
                continue
            
            nearest_target = self.find_nearest_target(trans)
            if self.nearest_target != nearest_target:
                rospy.loginfo("nearest target : {}".format(nearest_target))
                
            msg = String(data=nearest_target)
            self.pub.publish(msg)
            self.nearest_target = nearest_target
            self.rate.sleep()

    def find_nearest_target(self, cur_pos):
        nearest_target_name = None
        dist = None
        war_state_dict = self.converter(self.war_state)
        

        for target_name in TargetPositons.keys():
            if war_state_dict[target_name]['owner'] == self.war_state.my_side:
                continue

            target_pos = TargetPositons[target_name]
        
            # calc distance between target and current robot position (Squared distance)
            dist_i = (target_pos[0] - cur_pos[0])**2 + (target_pos[1]-cur_pos[1])**2

            if dist is None or dist_i < dist:
                nearest_target_name = target_name
                dist = dist_i
            
        return nearest_target_name

    def warStateCallback(self, msg):
        self.war_state = msg

    def converter(self, war_state):
        war_state_dict = {}

        for idx, target_name in enumerate(war_state.target_names):
            war_state_i = {}
            war_state_i['owner'] = war_state.target_owner[idx]
            war_state_i['point'] = war_state.target_point[idx]
            war_state_dict[target_name] = war_state_i
        return war_state_dict

if __name__ == '__main__':
    rospy.init_node('calc_nearest_target')
    bot = CalcNearestTargetBot()
    bot.strategy()
