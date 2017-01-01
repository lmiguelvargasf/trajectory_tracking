#!/usr/bin/env python

from trajectory.astroid_trajectory import AstroidTrajectory
from trajectory.circular_trajectory import CircularTrajectory
from trajectory.epitrochoid_trajectory import EpitrochoidTrajectory
from trajectory.lemniscate_trajectory import LemniscateTrajectory
from trajectory.linear_trajectory import LinearTrajectory
from trajectory.squared_trajectory import SquaredTrajectory

def create_trajectory(trajectory_name, simulation_time):
    if trajectory_name == 'linear':
        return LinearTrajectory(0.05, 0.01, 0.05, 0.01)
    elif trajectory_name == 'circular':
        return CircularTrajectory(2.0, simulation_time)
    elif trajectory_name == 'squared':
        return SquaredTrajectory(2.0, simulation_time, 0.01, 0.01)
    elif trajectory_name == 'astroid':
        return AstroidTrajectory(2.0, simulation_time)
    elif trajectory_name == 'lemniscate':
        return LemniscateTrajectory(2.0, simulation_time)
    elif trajectory_name == 'epitrochoid':
        return EpitrochoidTrajectory(5, 1, 3, simulation_time, 1 / 3.0)
