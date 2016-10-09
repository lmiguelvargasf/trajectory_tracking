#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from constants import DELTA_T, STEPS
from controller import Controller
from orientation import Orientation
from position import Position

current_pose = None
publisher = None
i = 0

controller = Controller()


def get_pose(message):
    global current_pose

    current_pose = message.pose[2]


def compute_control_actions(pose):
    global i

    current_position = Position.get_position_from_pose(pose)
    current_orientation = Orientation.get_orientation_from_pose(pose)

    current_reference = Position.get_position_at(i * DELTA_T)
    next_reference = Position.get_position_at((i + 1) * DELTA_T)

    theta_ez_n = controller.get_theta_ez_n(next_reference, current_reference, current_position)
    
    V_n = controller.get_V_n(next_reference, current_reference, current_position, theta_ez_n)
    w_n = controller.get_w_n(theta_ez_n, current_orientation)

    twist = Twist()
    twist.linear.x = V_n
    twist.angular.z = w_n

    publisher.publish(twist)

    i += 1


if __name__ == '__main__':
    rospy.init_node('control')
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    publisher = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    while current_pose is None:
        pass

    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown() and i < STEPS:
        compute_control_actions(current_pose)
        rate.sleep()

    rospy.spin()
