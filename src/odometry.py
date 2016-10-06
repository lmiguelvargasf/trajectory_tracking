#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    rospy.init_node('odometry')
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()
