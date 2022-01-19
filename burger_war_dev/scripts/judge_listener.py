#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is judge_listener node.
subscribe No topcs.
Publish 'judge_info' topic. 
mainly use for simple sample program

by yoda-ocu.
'''

import rospy
import rosparam
from std_msgs.msg import String
from burger_war_dev.msg import war_state

import requests
import json

class JudgeListener():
    def __init__(self,side, judge_url, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        self.side = side
        self.url = judge_url + "warState"
        self.war_state_pub = rospy.Publisher('war_state_info', war_state, queue_size=10)
        rospy.loginfo("College-Friends side is {}".format(self.side))

        if requests.get(self.url).ok:
            self.strategy()
        else:
            rospy.logerr("Failed to fecth warState : {}".format(self.url))

    def fetchState(self):
        resp = requests.get(self.url)           
        resp_json = resp.json()        

        state = war_state()
        state.time = resp_json['time']       
        state.my_side = self.side         
        state.my_point = resp_json['scores'][self.side]        
        state.target_names = []
        state.target_owner = []
        state.target_point = []

        for target in resp_json['targets']:            
            state.target_names.append(target['name'])
            state.target_owner.append(target['player'])                        
            state.target_point.append(int(target['point']))
            
        if self.side == 'r':
            state.enemy_point = resp_json['scores']['b']        
        elif self.side == 'b':
            state.enemy_point = resp_json['scores']['r']        
        else:            
            rospy.logerr("UNEXPECTED SIDE NAME : {}".format(self.side))
            raise Exception("UNEXPECTED SIDE NAME : {}".format(self.side))
            
        return state

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            info = self.fetchState()
            self.war_state_pub.publish(info)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('judge_node')
    bot = JudgeListener(side=rosparam.get_param("cf_side"), judge_url=rosparam.get_param("judge_url"))

