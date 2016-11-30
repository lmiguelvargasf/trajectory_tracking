#!/usr/bin/env python
import math

from geometry_msgs.msg import Point
from constants import SIMULATION_TIME_IN_SECONDS, TRAJECTORY


def create_trajectory():
    if TRAJECTORY == 'linear':
        return LinearTrajectory(0.05, 0.01, 0.05, 0.01)
    elif TRAJECTORY == 'circular':
        return CircularTrajectory(2.0, 120)
    elif TRAJECTORY == 'squared':
        return SquaredTrajectory(2.0, 0.01, 0.01)


class NegativeTimeException(Exception):
    pass

class Trajectory:
    def __init__(self):
        self.position = Point()

    def get_position_at(self, t):
        pass


class LinearTrajectory(Trajectory):
    def __init__(self, v_x, x_0=0, v_y=0, y_0=0):
        Trajectory.__init__(self)
        self.v_x = v_x
        self.v_y = v_y
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        if t < 0:
            raise NegativeTimeException()


        self.position.x = self.v_x * t + self.x_0
        self.position.y = self.v_y * t + self.y_0

        return self.position

class CircularTrajectory(Trajectory):
    def __init__(self, radius, period, x_0=0, y_0=0):
        Trajectory.__init__(self)
        self.radius = radius
        self. period = period
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        self.position.x = self.radius * math.sin(2 * math.pi * t / self.period) + self.x_0
        self.position.y = -self.radius * math.cos(2 * math.pi * t / self.period) + self.y_0

        return self.position


class SquaredTrajectory(Trajectory):
    def __init__(self, side, x_0=0, y_0=0):
        Trajectory.__init__(self)
        self.side = side
        self.v = 4.0 * side / SIMULATION_TIME_IN_SECONDS
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        if 0 <= t < SIMULATION_TIME_IN_SECONDS / 4:
            self.position.x = self.v * t + self.x_0
            self.position.y = self.y_0
        elif SIMULATION_TIME_IN_SECONDS / 4 <= t < SIMULATION_TIME_IN_SECONDS / 2:
            self.position.x = self.x_0 + self.side
            self.position.y = self.v * (t - SIMULATION_TIME_IN_SECONDS / 4) + self.y_0
        elif SIMULATION_TIME_IN_SECONDS / 2 <= t < 3 * SIMULATION_TIME_IN_SECONDS / 4:
            self.position.x = -self.v * (t - SIMULATION_TIME_IN_SECONDS / 2) + self.x_0 + self.side
            self.position.y = self.y_0 + self.side
        else:
            self.position.x = self.x_0
            self.position.y = -self.v * (t - 3 * SIMULATION_TIME_IN_SECONDS / 4) + self.y_0 + self.side

        return self.position
