#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

publisher = None

def send_computed_control_actions(msg):
    publisher.publish(msg)
    

if __name__ == '__main__':
    rospy.init_node('pose')
    subscriber = rospy.Subscriber('computed_pose', Odometry, send_computed_control_actions)
    publisher = rospy.Publisher('cmd_vel', Odometry, queue_size=1)
    rospy.spin()
