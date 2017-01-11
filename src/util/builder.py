#!/usr/bin/env python

from controller.euler_controller import EulerMethodController
from controller.pid_controller import PIDController


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
