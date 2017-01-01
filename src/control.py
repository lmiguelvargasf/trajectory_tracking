#!/usr/bin/env python
from __future__ import print_function

import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from plotter.simulation_plotter import SimulationPlotter
from util.util import create_trajectory, create_controller

DELTA_T = 0.1  # this is the sampling time
SIM_INFO = {
    'linear': (10.0, 0.075, 1.25),
    'circular': (120.0, 0.11, 1.25),
    'squared': (160.0, 0.11, 1.25),
    'astroid': (120.0, 0.105, 1.25),
    'lemniscate': (120.0, 0.125, 1.25),
    'epitrochoid': (240.0, 0.162, 1.25),
}


def get_pose(message):
    global current_pose, current_twist
    current_pose = message.pose[-1]
    current_twist = message.twist[-1]


def compute_control_actions():
    global i
    controller.compute_control_actions(current_pose, current_twist, i)
    plotter.add_point(current_pose)
    plotter.plot_data.theta.append(controller.theta_n)
    plotter.plot_data.theta_ref.append(controller.theta_ref_n)
    plotter.plot_data.v_c.append(controller.v_c_n)
    plotter.plot_data.w_c.append(controller.w_c_n)

    twist = Twist()
    twist.linear.x = controller.v_c_n
    twist.angular.z = controller.w_c_n
    twist_publisher.publish(twist)

    i += 1


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Error:', 'missing arguments!' if len(sys.argv) < 3 else 'too much arguments!')
        print('Try: rosrun trajectory_tracking control.py <controller> <trajectory>')
        sys.exit(-1)

    if sys.argv[1] in ('euler', 'pid'):
        CONTROLLER = sys.argv[1]
    else:
        print('Error: "{}" not valid controller name!'.format(sys.argv[1]))
        sys.exit(-2)

    if sys.argv[2] in ('linear', 'circular', 'squared'):
        TRAJECTORY = sys.argv[2]
        SIM_TIME = SIM_INFO[sys.argv[2]][0]
        STEPS = int(SIM_TIME / DELTA_T)
        MAX_V = SIM_INFO[sys.argv[2]][1]
        MAX_W = SIM_INFO[sys.argv[2]][2]
    else:
        print('Error: "{}" not a valid trajectory name!'.format(sys.argv[2]))
        sys.exit(-3)

    PATH_TO_EXPORT_DATA = '../txt_results/' + CONTROLLER + '/' + TRAJECTORY + '/'

    rospy.init_node('control')
    current_pose = None
    current_twist = None
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    twist_publisher = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    while current_pose is None or current_twist is None:
        pass

    i = 0
    trajectory = create_trajectory(TRAJECTORY, SIM_TIME)
    plotter = SimulationPlotter(trajectory, STEPS, DELTA_T, CONTROLLER, PATH_TO_EXPORT_DATA)
    controller = create_controller(trajectory, CONTROLLER, DELTA_T, SIM_INFO)
    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown() and i < STEPS:
        compute_control_actions()
        rate.sleep()

    print('Simulation was completed successfully!')

    # wait before plotting after simulation is completed
    time.sleep(2)

    plotter.plot_results()
    plotter.export_results()
    print ('Data was exported successfully!')

    rospy.spin()
