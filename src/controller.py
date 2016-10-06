#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose
from trajectory_tracking.srv import TrajectoryPoint
from constants import delta_t

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
    service_proxy = rospy.ServiceProxy('trajectory', TrajectoryPoint)
    
    current_reference = service_proxy(i * delta_t)
    next_reference = service_proxy((i + 1) * delta_t)


if __name__ == '__main__':
    rospy.init_node('controller')
    subscriber = rospy.Subscriber('pose_10_hz', Pose, compute_control_actions)
    rospy.wait_for_service('trajectory_server')
    rospy.spin()
