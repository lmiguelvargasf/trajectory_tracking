#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

current_odometry = None

def get_odometry(message):
    global current_odometry
    current_odometry = message


if __name__ == '__main__':
    rospy.init_node('odometry')
    subscriber = rospy.Subscriber('odom', Odometry, get_odometry)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
