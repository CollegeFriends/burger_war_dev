#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import tf


from actionlib_msgs.msg import GoalStatusArray

from local_map import *


class NaviBot():
    DISTANCE = 0.25

    def __init__(self):
        self.sub = rospy.Subscriber('nearest_target', String, self.nearestTargetCallback)
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=5)
        self.sub_status = rospy.Subscriber('move_base/status', GoalStatusArray, self.navStateCallback)    

        self.navi_status = None
        self.target_name = None
        
    def setGoal(self,goal):
        # self.client.wait_for_server()
        # self.client.send_goal(goal)
        self.last_send_goal = goal
        self.pub_goal.publish(goal)

    def cancelGoal(self):
        rospy.loginfo("Cancel all goals")
        # self.client.cancel_goal()

    def nearestTargetCallback(self, msg):
        self.target_name = msg.data
        rospy.loginfo("Nearest targe is {}".format(self.target_name))
        
    def strategy(self):
        self.rate = rospy.Rate(1)
        self.last_target_name = None
        self.last_send_goal = None

        while not rospy.is_shutdown():
            target = self.target_name 
            try:
                if target != self.last_target_name:
                    
                    goal = self.calcGoal(target)
                    
                    if goal is None:
                        raise ValueError(target)
                    self.cancelGoal()
                    rospy.loginfo("Set Goal to {} [{}]".format(target, goal.pose.position))
                    self.setGoal(goal)
                    # rospy.loginfo("Result {}".format(ret))
                    self.last_target_name = target
                

            except:
                rospy.logerr("ERROR {}".format(target))
                pass
            finally:
                self.rate.sleep()

    def calcGoal(self, targetName):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        
        x=0.0
        y=0.0
        yaw=0.0

        if targetName not in TargetPositons.keys():
            rospy.logwarn("Unexpected targetName : {} ".format(targetName))
            return None

        x, y = TargetPositons[targetName]
        q = [0.0, 0.0, 0.0, 1.0]

        if targetName[-1] == "N":
            x += self.DISTANCE
            q[2] = 1.0
            q[3] = 0.0
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
            rospy.logerr("Not implemented target name {}".format(targetName))
            return None

        
        goal.pose.position.x = x
        goal.pose.position.y = y      
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]        
        
        return goal

    def navStateCallback(self, data):
        
        if len(data.status_list) > 0:
            status = data.status_list[0]
            if status == self.navi_status:
                return
            self.navi_status = status
            rospy.loginfo("Navi Status : {}".format(status))

            if status.status == 3:
                rospy.logwarn("Reached but not take target == LOST!")


if __name__ == '__main__':
    rospy.init_node('decide_move_goal')
    bot = NaviBot()
    bot.strategy()
