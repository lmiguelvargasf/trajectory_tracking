#!/usr/bin/env python
from __future__ import print_function

import os
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from plotter.plotter import get_data_container
from plotter.simulation_plotter import SimulationPlotter
from trajectory.builder import create_trajectory
from util.builder import create_controller
from util.results import export_results

DELTA_T = 0.1  # this is the sampling time
SIM_INFO = {
    'linear': (60.0, 0.075, 1.25),
    'circular': (120.0, 0.11, 1.25),
    'squared': (160.0, 0.11, 1.25),
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
    data_container['t'].append(i * DELTA_T)
    data_container['x'].append(current_pose.position.x)
    data_container['x_ref'].append(controller.x_ref_n)
    data_container['y'].append(current_pose.position.y)
    data_container['y_ref'].append(controller.y_ref_n)
    data_container['theta'].append(controller.theta_n)
    data_container['theta_ref'].append(controller.theta_ref_n)
    data_container['v_c'].append(controller.v_c_n)
    data_container['w_c'].append(controller.w_c_n)
    data_container['zeros'].append(0)

    twist = Twist()
    twist.linear.x = controller.v_c_n
    twist.angular.z = controller.w_c_n
    twist_publisher.publish(twist)

    i += 1


if __name__ == '__main__':
    parameters = sys.argv[1:]
    if len(parameters) not in (2, 3):
        print('Error:', end=' ')
        if len(parameters) < 2:
            print('missing arguments!')
        elif len(parameters) > 3:
            print('too much arguments!')

        print('Try: rosrun trajectory_tracking control.py <controller> <trajectory> [simulation_time]')
        sys.exit(-1)

    if parameters[0] in ('euler', 'pid'):
        CONTROLLER = parameters[0]
    else:
        print('Error: "{}" not valid controller name!'.format(sys.argv[1]))
        sys.exit(-2)

    if parameters[1] in ('linear', 'circular', 'squared'):
        TRAJECTORY = parameters[1]
        PERIOD = SIM_INFO[TRAJECTORY][0]
        MAX_V = SIM_INFO[TRAJECTORY][1]
        MAX_W = SIM_INFO[TRAJECTORY][2]
    else:
        print('Error: "{}" not a valid trajectory name!'.format(parameters[1]))
        sys.exit(-3)

    try:
        SIM_TIME = float(parameters[2])

        if SIM_TIME <= 0:
            print('Error: simulation time must be a positive number!')
            sys.exit(-4)

    except ValueError:
        print('Error: simulation time must be a valid number!')
        sys.exit(-5)
    except IndexError:
        SIM_TIME = PERIOD

    STEPS = int(SIM_TIME / DELTA_T)

    rospy.init_node('control')
    current_pose = None
    current_twist = None
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    twist_publisher = rospy.Publisher('computed_control_actions', Twist, queue_size=1)

    while current_pose is None or current_twist is None:
        pass

    i = 0
    trajectory = create_trajectory(TRAJECTORY, PERIOD)
    data_container = get_data_container(CONTROLLER)
    controller = create_controller(trajectory, CONTROLLER, DELTA_T, SIM_INFO)
    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown() and i < STEPS:
        compute_control_actions()
        rate.sleep()

    print('Simulation was completed successfully!')
    export_results(data_container,
                   CONTROLLER, TRAJECTORY,
                   os.sep.join(__file__.split(os.sep)[:-2]) + '/results.db')
    print('Data was exported successfully!')
    print('Plotting results...')
    plotter = SimulationPlotter(data_container)
    plotter.plot_results()
    print('Press Ctrl+C to terminate program.')

    rospy.spin()
