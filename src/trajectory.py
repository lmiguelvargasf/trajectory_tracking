#!/usr/bin/env python
import math
from geometry_msgs.msg import Point


def create_trajectory(trajectory_type):
    if trajectory_type == 'linear':
        return LinearTrajectory(0.05, 0, 0.05, 0)
    elif trajectory_type == 'circular':
        return CircularTrajectory(2.0, 120)


class Trajectory:
    def get_position_at(self, t):
        pass


class LinearTrajectory(Trajectory):
    def __init__(self, v_x, x_0, v_y, y_0):
        self.v_x = v_x
        self.v_y = v_y
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        position = Point()
        position.x = self.v_x * t + self.x_0
        position.y = self.v_y * t + self.y_0

        return position

class CircularTrajectory(Trajectory):
    def __init__(self, radius, period):
        self.radius = radius
        self. period = period

    def get_position_at(self, t):
        position = Point()
        position.x = self.radius * math.cos(2 * math.pi * t / self.period)
        position.y = self.radius * math.sin(2 * math.pi * t / self.period)

        return position
