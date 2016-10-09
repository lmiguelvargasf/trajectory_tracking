#!/usr/bin/env python
import rospy
import math

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from constants import K_V, K_W, DELTA_T
from position import Position
from orientation import Orientation

current_pose = None
publisher = None
i = 0
theta_ez_n_minus_1 = 0
theta_n_minus_1 = 0


def get_pose(message):
    global current_pose

    current_pose = message.pose[2]


def get_theta_ez_n(next_reference, current_reference, current_position):
    x_ref_n_plus_1 = next_reference.x
    y_ref_n_plus_1 = next_reference.y
    x_ref_n = current_reference.x
    y_ref_n = current_reference.y
    x_n = current_position.x
    y_n = current_position.y
    
    numerator = y_ref_n_plus_1 - K_V * (y_ref_n - y_n) - y_n
    denominator = x_ref_n_plus_1 - K_V * (x_ref_n - x_n) - x_n
    
    return math.atan2(numerator, denominator)


def get_V_n(next_reference, current_reference, current_position, theta_ez_n):
    x_ref_n_plus_1 = next_reference.x
    y_ref_n_plus_1 = next_reference.y
    x_ref_n = current_reference.x
    y_ref_n = current_reference.y
    x_n = current_position.x
    y_n = current_position.y
    
    operand_0 = x_ref_n_plus_1 - K_V * (x_ref_n - x_n) - x_n
    operand_1 = y_ref_n_plus_1 - K_V * (y_ref_n - y_n) - y_n
    
    return (operand_0 * math.cos(theta_ez_n) + operand_1 * math.sin(theta_ez_n)) / DELTA_T


def get_w_n(theta_ez_n, current_orientation):
    global theta_ez_n_minus_1, theta_n_minus_1
    
    w_n = (theta_ez_n - K_W * (theta_ez_n_minus_1 - theta_n_minus_1) - theta_n_minus_1) / DELTA_T
    
    theta_ez_n_minus_1 = theta_ez_n
    theta_n_minus_1 = current_orientation[2]
    
    return math.atan2(math.sin(w_n), math.cos(w_n))


def compute_control_actions(pose):
    global i

    current_position = Position.get_position_from_pose(pose)
    current_orientation = Orientation.get_orientation_from_pose(pose)

    current_reference = Position.get_position_at(i * DELTA_T)
    next_reference = Position.get_position_at((i + 1) * DELTA_T)

    theta_ez_n = get_theta_ez_n(next_reference, current_reference, current_position)
    
    V_n = get_V_n(next_reference, current_reference, current_position, theta_ez_n)
    w_n = get_w_n(theta_ez_n, current_orientation)

    twist = Twist()
    twist.linear.x = V_n
    twist.angular.z = w_n

    publisher.publish(twist)

    i += 1


if __name__ == '__main__':
    rospy.init_node('controller')
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    publisher = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    while current_pose is None:
        pass

    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown():
        compute_control_actions(current_pose)
        rate.sleep()
