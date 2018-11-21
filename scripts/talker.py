#!/usr/bin/env python

import sys
import pandas as pd

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


"""
This class acts as a container for the cartesian velocities.
Iterator functions are implemented, this class can be used in
a for loop to iterate through all elements.
"""
class velocity_container():
    def __init__(self,df):
        self.data = df.to_dict()
        self.length = len(df)
        self.index = 0
        print('Velocity container initialized, data length is : {0}'.format(self.length))

    def __iter__(self):
        return self

    def next(self):
        if self.index >= self.length:
            raise StopIteration
        else:
            vx = self.data['vx'][self.index]
            vy = self.data['vy'][self.index]
            vomega = self.data['omega'][self.index]
            self.index+=1
            print 'next called with :{0},{1},{2}'.format(vx,vy,vomega)
            return vx,vy,vomega


def talker(velocity_data):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    for vx,vy,omega in velocity_data:
        if rospy.is_shutdown():
            break
        else:
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = 0

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = omega
            pub.publish(twist)
            rospy.loginfo(twist)
            pub.publish(twist)
            rate.sleep()
    stop_command = Twist()
    pub.publish(stop_command)
    rospy.loginfo(stop_command)
    pub.publish(stop_command)

if __name__ == '__main__':
    try:
        arguments = sys.argv[1:]
        print('launched with argument:', arguments)
        if not arguments:
			print("\033[91m Missing argument for the input file containing the velocities!\033[0m")
        else:
			loaded_df = pd.read_csv(arguments[0])
			velocities = velocity_container(loaded_df)
			print('Dataframe loaded! Starting publishing')
			talker(velocities)
    except rospy.ROSInterruptException:
        pass
