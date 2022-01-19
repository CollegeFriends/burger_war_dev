#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is decide_target node.
Subscribe tf.
Publish 'nearest_target' topic. 
mainly use for simple sample program

by yoda-ocu.
'''

from operator import ne
import rospy
from std_msgs.msg import String
import tf

class DecideTargetBot():
    TargetPositons = {
        "FriedShrimp_N":    [+0.35/2.0,         0.0],
        "FriedShrimp_S":    [-0.35/2.0,         0.0],
        "FriedShrimp_E":    [0.0,               -0.35/2.0],
        "FriedShrimp_W":    [0.0,               +0.35/2.0],
        "Omelette_N":       [+0.53+0.15/2.0,    -0.53],
        "Omelette_S":       [+0.53-0.15/2.0,    -0.53],
        "Tomato_N":         [+0.53+0.15/2.0,    +0.53],
        "Tomato_S":         [+0.53-0.15/2.0,    +0.53],
        "OctopusWiener_N":  [-0.53+0.15/2.0,    -0.53],
        "OctopusWiener_S":  [-0.53-0.15/2.0,    -0.53],
        "Pudding_N":        [-0.53+0.15/2.0,    +0.53],
        "Pudding_S":        [-0.53-0.15/2.0,    +0.53],
    }

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher("nearest_target",String, queue_size=10)

    def strategy(self):
        base_time = rospy.Time.now()

        self.nearest_target = None

        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                # rospy.loginfo("trans:{} rot:{}".format(trans,rot))
                nearest_target = self.find_nearest_target(trans)
                
                if self.nearest_target != nearest_target:
                    rospy.loginfo("nearest target : {}".format(nearest_target))
                    msg = String(data=nearest_target)
                    self.pub.publish(msg)
                    self.nearest_target = nearest_target
            except:
                rospy.loginfo("Wait for tf")
            finally:
                self.rate.sleep()

    def find_nearest_target(self, cur_pos):
        nearest_target_name = None
        dist = None

        for target_name in self.TargetPositons.keys():
            target_pos = self.TargetPositons[target_name]
        
            # calc distance between target and current robot position (Squared distance)
            dist_i = (target_pos[0] - cur_pos[0])**2 + (target_pos[1]-cur_pos[1])**2

            if dist is None or dist < dist_i:
                nearest_target_name = target_name
            
        return nearest_target_name

if __name__ == '__main__':
    rospy.init_node('decide_target')
    bot = DecideTargetBot()
    bot.strategy()
