#!/usr/bin/env python
# -*- coding: utf-8 -*-
from curses import qiflush
import rospy
import math

from std_msgs.msg import String

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from local_map import *


class NaviBot():
    DISTANCE = 0.25

    def __init__(self):
        self.sub = rospy.Subscriber('nearest_target', String, self.nearestTargetCallback)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
    def setGoal(self,goal):
        self.client.wait_for_server()
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def nearestTargetCallback(self, msg):
        self.target_name = msg.data
        
    def strategy(self):
        self.rate = rospy.Rate(1)
        self.last_target_name = None
        while not rospy.is_shutdown():
            try:
                if self.target_name != self.last_target_name:
                    goal = self.calcGoal(self.target_name)
                    rospy.loginfo("Set Goal to {} [{}]".format(self.target_name, goal.position))
                    ret = self.setGoal(goal)
                    rospy.loginfo("Result {}".format(ret))
                    self.last_target_name = self.target_name
            except:
                pass
            finally:
                self.rate.sleep()

    def calcGoal(self, targetName):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        

        x=0.0
        y=0.0
        yaw=0.0

        x, y = TargetPositons[targetName]
        q = [0.0, 0.0, 0.0, 1.0]

        if targetName[-1] == "N":
            x += self.DISTANCE
            q[2] = 1.0
            qiflush[3] = 0.0
        elif targetName[-1] == "S":
            x -= self.DISTANCE
            q[2] = 0.0
            q[3] = 1.0
        elif targetName[-1] == "W":
            y += self.DISTANCE
            q[2] = math.sin(-math.pi/4.0)
            q[3] = math.cos(-math.pi/4.0)
        elif targetName[-1] == "E":
            y -= self.DISTANCE
            q[2] = math.sin(math.pi/4.0)
            q[3] = math.cos(math.pi/4.0)
        else:
            rospy.logerror("Not implemented target name {}".format(targetName))
            return None

        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y      
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]        

        return goal


if __name__ == '__main__':
    rospy.init_node('decide_move_goal')
    bot = NaviBot()
    bot.strategy()
