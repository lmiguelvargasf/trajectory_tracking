#!/usr/bin/env python
from __future__ import print_function
import rospy
import time
import sys
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from constants import DELTA_T, STEPS, K_X, K_Y, K_THETA, SIMULATION_TIME_IN_SECONDS, K_P_V, K_I_V, K_D_V, \
    K_P_W, K_I_W, K_D_W, MAX_V, MAX_W, TRAJECTORY, RESULTS_DIRECTORY
from controller.euler_controller import EulerMethodController
from controller.pid_controller import PIDController
from plotter.simulation_plotter import SimulationPlotter
from trajectory.astroid_trajectory import AstroidTrajectory
from trajectory.circular_trajectory import CircularTrajectory
from trajectory.epitrochoid_trajectory import EpitrochoidTrajectory
from trajectory.lemniscate_trajectory import LemniscateTrajectory
from trajectory.linear_trajectory import LinearTrajectory
from trajectory.squared_trajectory import SquaredTrajectory


def create_controller(trajectory):
    simulation_data = {'delta': DELTA_T, 'time': SIMULATION_TIME_IN_SECONDS}
    if CONTROLLER == 'euler':
        return EulerMethodController(
            trajectory,
            simulation_data,
            {'x': K_X, 'y': K_Y, 'theta': K_THETA}
        )
    elif CONTROLLER == 'pid':
        return PIDController(
            trajectory,
            simulation_data,
            {'kpv': K_P_V, 'kiv': K_I_V, 'kdv': K_D_V, 'kpw': K_P_W, 'kiw': K_I_W, 'kdw': K_D_W},
            {'linear': MAX_V, 'angular': MAX_W}
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
    if len(sys.argv) != 2:
        print('Error:', 'missing arguments!' if len(sys.argv) < 2 else 'too much arguments!')
        print('Try: rosrun trajectory_tracking control.py <controller>')
        sys.exit(-1)

    if sys.argv[1] in ('euler', 'pid'):
        CONTROLLER = sys.argv[1]
        PATH_TO_EXPORT_DATA = RESULTS_DIRECTORY + CONTROLLER + '/' + TRAJECTORY + '/'
    else:
        print('Error: "{}" not valid controller name!'.format(sys.argv[1]))
        sys.exit(-2)

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
