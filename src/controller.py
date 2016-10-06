#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

i = 0

def get_position(pose):
    return pose.pose.position

def get_orientation(pose):
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
        )
    return tf.transformations.euler_from_quaternion(quaternion)

def compute_control_actions(msg):
    global i
    pose = msg.pose
    
    current_position = get_position(pose)
    current_orientation = get_orientation(pose)


if __name__ == '__main__':
    rospy.init_node('controller')
    subscriber = rospy.Subscriber('odometry_10_hz', Odometry, compute_control_actions)
    rospy.spin()
