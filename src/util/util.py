#!/usr/bin/env python
from constants import TRAJECTORY, SIMULATION_TIME_IN_SECONDS
from trajectory.astroid_trajectory import AstroidTrajectory
from trajectory.circular_trajectory import CircularTrajectory
from trajectory.linear_trajectory import LinearTrajectory
from trajectory.squared_trajectory import SquaredTrajectory


def create_trajectory():
    if TRAJECTORY == 'linear':
        return LinearTrajectory(0.05, 0.01, 0.05, 0.01)
    elif TRAJECTORY == 'circular':
        return CircularTrajectory(2.0, SIMULATION_TIME_IN_SECONDS)
    elif TRAJECTORY == 'squared':
        return SquaredTrajectory(2.0, SIMULATION_TIME_IN_SECONDS, 0.01, 0.01)
    elif TRAJECTORY == 'astroid':
        return AstroidTrajectory(2.0, SIMULATION_TIME_IN_SECONDS)
