#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Pose
from trajectory_tracking.srv import TrajectoryPoint
from constants import delta_t, k_V, k_w

i = 0


def get_position(pose):
    return pose.position

def get_orientation(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
        )

    return tf.transformations.euler_from_quaternion(quaternion)


def get_theta_ez_n(next_reference, current_reference, current_position):
    x_ref_n_plus_1 = next_reference.x
    y_ref_n_plus_1 = next_reference.y
    x_ref_n = current_reference.x
    y_ref_n = current_reference.y
    x_n = current_position.x
    y_n = current_position.y
    
    numerator = y_ref_n_plus_1 - k_V * (y_ref_n - y_n) - y_n
    denominator = x_ref_n_plus_1 - k_V * (x_ref_n - x_n) - x_n
    
    return math.atan2(numerator, denominator)


def compute_control_actions(pose):
    global i
    
    current_position = get_position(pose)
    current_orientation = get_orientation(pose)
    service_proxy = rospy.ServiceProxy('trajectory', TrajectoryPoint)
    
    current_reference = service_proxy(i * delta_t).position
    next_reference = service_proxy((i + 1) * delta_t).position
    
    theta_ez_n = get_theta_ez_n(next_reference, current_reference, current_position)

    i += 1


if __name__ == '__main__':
    rospy.init_node('controller')
    subscriber = rospy.Subscriber('pose_10_hz', Pose, compute_control_actions)
    rospy.wait_for_service('trajectory_server')
    rospy.spin()
