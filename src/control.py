#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from constants import DELTA_T, STEPS
from controller import Controller


def get_pose(message):
    global current_pose
    current_pose = message.pose[2]


def compute_control_actions(pose):
    global i
    controller.compute_control_actions(pose, i)

    twist = Twist()
    twist.linear.x = controller.v_n
    twist.angular.z = controller.w_n
    publisher.publish(twist)

    i += 1


if __name__ == '__main__':
    rospy.init_node('control')
    current_pose = None
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    publisher = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    while current_pose is None:
        pass

    i = 0
    controller = Controller()
    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown() and i < STEPS:
        compute_control_actions(current_pose)
        rate.sleep()

    rospy.spin()
