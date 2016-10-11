#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose

from constants import DELTA_T, STEPS
from controller import EulerMethodController, create_controller
from plotter import Plotter


def get_pose(message):
    global current_pose
    current_pose = message.pose[2]


def compute_control_actions(pose):
    global i
    controller.compute_control_actions(pose, i)
    plotter.add_point(pose)

    twist = Twist()
    twist.linear.x = controller.v_n
    twist.angular.z = controller.w_n
    twist_publisher.publish(twist)

    i += 1


if __name__ == '__main__':
    rospy.init_node('control')
    current_pose = None
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    twist_publisher = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    while current_pose is None:
        pass

    i = 0
    plotter = Plotter()
    controller = create_controller()
    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown() and i < STEPS:
        compute_control_actions(current_pose)
        rate.sleep()

    plotter.plot_results()
    rospy.spin()
