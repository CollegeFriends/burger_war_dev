#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool
from burger_war_dev.msg import war_state
from actionlib_msgs.msg import GoalStatusArray

class StateControlBot():
    def __init__(self):
        self.pub = rospy.Publisher("main_state",String, queue_size=10)
        self.sub = rospy.Subscriber("war_state_info", war_state, self.warStateCallback)
        self.sub_navi_status = rospy.Subscriber('move_base/status', GoalStatusArray, self.navStateCallback)    
        self.sub_detectingEnemy = rospy.Subscriber('detect_enemy', Bool, self.detectEnemyCallback)
        self.detecting_enemy = False
        self.detected_time = None
        self.state = "UNDEFINED"
        self.navi_status = None
        self.war_state = war_state()

    def strategy(self):
        self.publish_state("IDLING")
        self.rate = rospy.Rate(1)        
        while not rospy.is_shutdown():
            if self.state == "IDLING":
                if self.war_state.state == "running":
                    self.publish_state("GO")
            elif self.war_state.state == "stop":
                if self.war_state.my_point < self.war_state.enemy_point:
                    self.publish_state("LOSE")
                elif self.war_state.my_point > self.war_state.enemy_point:
                    self.publish_state("WIN")
                else:
                    self.publish_state("EVEN")
            elif self.state == "GO" and self.detecting_enemy:
                self.publish_state("ESCAPE")
                rospy.sleep(rospy.Duration(10))
                self.publish_state("GO")

            self.rate.sleep()
            
    def navStateCallback(self, data):
        if len(data.status_list) > 0:
            status = data.status_list[0]
            if status == self.navi_status:
                return
            self.navi_status = status
            rospy.logdebug("Navi Status : {}".format(status))

    def detectEnemyCallback(self,msg):
        self.detecting_enemy = msg.data

    def publish_state(self, state):
        rospy.loginfo("STATE : {}".format(state))
        self.state = state
        msg = String(data=state)
        self.pub.publish(msg)

    def warStateCallback(self, msg):
        self.war_state = msg
        rospy.logdebug("msg.state {}".format(msg.state))

def main():
    rospy.init_node('state_control')
    bot = StateControlBot()
    bot.strategy()

if __name__ == "__main__":
    main()