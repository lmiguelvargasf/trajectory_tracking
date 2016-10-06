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
    publisher = rospy.Publisher('odometry_10_hz', Odometry, queue_size=1)
    
    while current_odometry == None:
        pass
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publisher.publish(current_odometry)
        rate.sleep()
