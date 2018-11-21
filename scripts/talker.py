#!/usr/bin/env python

import sys
import pandas as pd

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 40, fill = '#'):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix))
    # Print New Line on Complete
    if iteration == total: 
        print()


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

    def __iter__(self):
        return self

    def next(self):
        if self.index >= self.length:
			print("Execution finsihed!")
			raise StopIteration
        else:
            vx = self.data['vx'][self.index]
            vy = self.data['vy'][self.index]
            vomega = self.data['omega'][self.index]
            self.index+=1
            printProgressBar(self.index, self.length, suffix = 'vx:{0:.3f}|vy:{1:.3f}|omega{2:.3f}'.format(float(vx),float(vy),float(vomega)))
            return vx,vy,vomega


def talker(velocity_data):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True, log_level=rospy.WARN)
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
