#!/usr/bin/env python
import math

from geometry_msgs.msg import Point


class NegativeTimeException(Exception):
    pass

class Trajectory:
    def __init__(self):
        self.position = Point()

    def get_position_at(self, t):
        if t < 0:
            raise NegativeTimeException()

class CircularTrajectory(object, Trajectory):
    def __init__(self, radius, period, x_0=0, y_0=0):
        Trajectory.__init__(self)
        self.radius = radius
        self. period = period
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        super(CircularTrajectory, self).get_position_at(t)
        self.position.x = self.radius * math.sin(2 * math.pi * t / self.period) + self.x_0
        self.position.y = -self.radius * math.cos(2 * math.pi * t / self.period) + self.y_0

        return self.position


class SquaredTrajectory(object, Trajectory):
    def __init__(self, side, period, x_0=0, y_0=0):
        Trajectory.__init__(self)
        self.side = side
        self.period = period
        self.v = 4.0 * side / period
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        super(SquaredTrajectory, self).get_position_at(t)

        if 0 <= t < self.period / 4:
            self.position.x = self.v * t + self.x_0
            self.position.y = self.y_0
        elif self.period / 4 <= t < self.period / 2:
            self.position.x = self.x_0 + self.side
            self.position.y = self.v * (t - self.period / 4) + self.y_0
        elif self.period / 2 <= t < 3 * self.period / 4:
            self.position.x = -self.v * (t - self.period / 2) + self.x_0 + self.side
            self.position.y = self.y_0 + self.side
        else:
            self.position.x = self.x_0
            self.position.y = -self.v * (t - 3 * self.period / 4) + self.y_0 + self.side

        return self.position
