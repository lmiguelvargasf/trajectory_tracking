#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose
from trajectory_tracking.srv import TrajectoryPoint

i = 0

def get_position(pose):
    return pose.position

def get_orientation(pose):
    quaternion = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
        ]
    return tf.transformations.euler_from_quaternion(quaternion)

def compute_control_actions(pose):
    global i
    
    current_position = get_position(pose)
    current_orientation = get_orientation(pose)


if __name__ == '__main__':
    rospy.init_node('controller')
    subscriber = rospy.Subscriber('pose_10_hz', Pose, compute_control_actions)
    rospy.spin()
