#!/usr/bin/env python

import sys
import pandas as pd

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class velocity_container():
    def __init__(self,df):
        self.data = df
        self.length = len(self.data)
        self.index = 0

    def getNext(self):
        vx = self.data.iloc[self.index]['vx']
        vy = self.data.iloc[self.index]['vy']
        vomega = self.data.iloc[self.index]['omega']
        self.index += 1
        return vx,vy,vomega


def talker():
    pub = rospy.Publisher('v_cmd', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        x = 1
        y = 2
        speed = 1
        th = 0.1
        turn = 1
        twist = Twist()
        twist.linear.x = x*speed
        twist.linear.y = y*speed
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = th*turn
        pub.publish(twist)
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        arguments = sys.argv[1:]
        talker()
    except rospy.ROSInterruptException:
        pass
