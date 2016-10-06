#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

publisher = None

def send_computed_control_actions(msg):
    publisher.publish(msg)
    

if __name__ == '__main__':
    rospy.init_node('twist')
    subscriber = rospy.Subscriber('computed_control_actions', Twist, send_computed_control_actions)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()
