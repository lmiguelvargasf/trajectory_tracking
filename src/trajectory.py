#!/usr/bin/env python
from geometry_msgs.msg import Point


def create_trajectory(trajectory_type):
    if trajectory_type == 'linear':
        return LinearTrajectory(0.05, 0, 0.05, 0)


class LinearTrajectory:
    def __init__(self, v_x, x_0, v_y, y_0):
        self.v_x = v_x
        self.v_y = v_y
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_a(self, t):
        position = Point()
        position.x = self.v_x * t + self.x_0
        position.y = self.v_y * t + self.y_0

        return position
