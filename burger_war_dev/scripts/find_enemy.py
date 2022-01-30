#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool
from burger_war_dev.msg import rects

class FindEnemyBot():
    def __init__(self):
        self.detect_enemy_pub = rospy.Publisher("detect_enemy", Bool, queue_size=10)
        self.cvrect_sub = rospy.Subscriber("cv_rect", rects, self.rect_callback)
        self.isDetectingEnemy = False

    def rect_callback(self, msg):
        enemy_r = msg.rect_r
        enemy_g = msg.rect_g

        isDetecting = ((enemy_g.center != (-1.0, -1.0) and enemy_g.length[1] > 100.0) or (enemy_r.center != (-1.0, -1.0) and enemy_r.length[0] > 60.0))
        if isDetecting != self.isDetectingEnemy:
            self.isDetectingEnemy = isDetecting
            msg = Bool(data=self.isDetectingEnemy)
            self.detect_enemy_pub.publish(msg)

def main():
    rospy.init_node("find_enemy")
    bot = FindEnemyBot()
    rospy.spin()
    
if __name__ == '__main__':
    main()