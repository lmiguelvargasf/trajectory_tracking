#!/usr/bin/env python
import rospy

def compute_control_actions(msg):
    pass

if __name__ == '__main__':
    rospy.init_node('controller')
    subscriber = rospy.Subscriber('odometry_10_hz', Odometry, compute_control_actions)
    rospy.spin()
