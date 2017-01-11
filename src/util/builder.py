#!/usr/bin/env python

from controller.euler_controller import EulerMethodController
from controller.pid_controller import PIDController
from trajectory.circular_trajectory import CircularTrajectory
from trajectory.epitrochoid_trajectory import EpitrochoidTrajectory
from trajectory.lemniscate_trajectory import LemniscateTrajectory
from trajectory.linear_trajectory import LinearTrajectory
from trajectory.squared_trajectory import SquaredTrajectory



def create_trajectory(trajectory_name, period):
    if trajectory_name == 'linear':
        return LinearTrajectory(0.05, 0.01, 0.05, 0.01)
    elif trajectory_name == 'circular':
        return CircularTrajectory(2.0, period)
    elif trajectory_name == 'squared':
        return SquaredTrajectory(2.0, period, 0.01, 0.01)
    elif trajectory_name == 'lemniscate':
        return LemniscateTrajectory(2.0, period)
    elif trajectory_name == 'epitrochoid':
        return EpitrochoidTrajectory(5, 1, 3, period, 1 / 3.0)


def create_controller(trajectory, controller_name, delta, sim_info):
    simulation_data = {'delta': delta, 'time': sim_info[trajectory.get_name()][0]}
    if controller_name == 'euler':
        return EulerMethodController(
            trajectory,
            simulation_data,
            {'x': 0.9, 'y': 0.9, 'theta': 0.9}
        )
    elif controller_name == 'pid':
        return PIDController(
            trajectory,
            simulation_data,
            {'kpv': 0.2, 'kiv': 1.905, 'kdv': 0.00, 'kpw': 0.45, 'kiw': 1.25, 'kdw': 0.00},
            {'linear': sim_info[trajectory.get_name()][1], 'angular': sim_info[trajectory.get_name()][2]}
        )
