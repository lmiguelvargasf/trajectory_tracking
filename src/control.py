#!/usr/bin/env python
from __future__ import print_function

import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from controller.euler_controller import EulerMethodController
from controller.pid_controller import PIDController
from plotter.simulation_plotter import SimulationPlotter
from trajectory.astroid_trajectory import AstroidTrajectory
from trajectory.circular_trajectory import CircularTrajectory
from trajectory.epitrochoid_trajectory import EpitrochoidTrajectory
from trajectory.lemniscate_trajectory import LemniscateTrajectory
from trajectory.linear_trajectory import LinearTrajectory
from trajectory.squared_trajectory import SquaredTrajectory

DELTA_T = 0.1 # this is the sampling time

SIM_INFO = {
    'linear': (10.0, 0.075, 1.25),
    'circular': (120.0, 0.11, 1.25),
    'squared': (160.0, 0.11, 1,25),
    'astroid': (120.0, 0.105, 1.25),
    'lemniscate': (120.0, 0.125, 1.25),
    'epitrochoid': (240.0, 0.162, 1.25),
}

def create_controller(trajectory):
    simulation_data = {'delta': DELTA_T, 'time': SIMULATION_TIME_IN_SECONDS}
    if CONTROLLER == 'euler':
        return EulerMethodController(
            trajectory,
            simulation_data,
            {'x': 0.9, 'y': 0.9, 'theta': 0.9}
        )
    elif CONTROLLER == 'pid':
        return PIDController(
            trajectory,
            simulation_data,
            {'kpv': 0.2, 'kiv': 1.905, 'kdv': 0.00, 'kpw': 0.45, 'kiw': 1.25, 'kdw': 0.00},
            {'linear': SIM_INFO[trajectory.get_name()][1], 'angular': SIM_INFO[trajectory.get_name()][2]}
        )

def create_trajectory():
    if TRAJECTORY == 'linear':
        return LinearTrajectory(0.05, 0.01, 0.05, 0.01)
    elif TRAJECTORY == 'circular':
        return CircularTrajectory(2.0, SIMULATION_TIME_IN_SECONDS)
    elif TRAJECTORY == 'squared':
        return SquaredTrajectory(2.0, SIMULATION_TIME_IN_SECONDS, 0.01, 0.01)
    elif TRAJECTORY == 'astroid':
        return AstroidTrajectory(2.0, SIMULATION_TIME_IN_SECONDS)
    elif TRAJECTORY == 'lemniscate':
        return LemniscateTrajectory(2.0, SIMULATION_TIME_IN_SECONDS)
    elif TRAJECTORY == 'epitrochoid':
        return EpitrochoidTrajectory(5, 1, 3, SIMULATION_TIME_IN_SECONDS, 1 / 3.0)


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
        SIMULATION_TIME_IN_SECONDS = SIM_INFO[sys.argv[2]][0]
        STEPS = int(SIMULATION_TIME_IN_SECONDS / DELTA_T)
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
    trajectory = create_trajectory()
    plotter = SimulationPlotter(trajectory, STEPS, DELTA_T, CONTROLLER, PATH_TO_EXPORT_DATA)
    controller = create_controller(trajectory)
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
