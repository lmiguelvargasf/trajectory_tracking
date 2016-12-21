#!/usr/bin/env python
from math import cos, pi, sin

from .trajectory import Trajectory


class AstroidTrajectory(object, Trajectory):
    def __init__(self, radius, period):
        Trajectory.__init__(self)
        self.radius = radius
        self.period = period

    def get_position_at(self, t):
        super(AstroidTrajectory, self).get_position_at(t)
        self.position.x = self.radius * cos(2 * pi * t / self.period) ** 3
        self.position.y = self.radius * sin(2 * pi * t / self.period) ** 3

        return self.position

    def get_name(self):
        return str(AstroidTrajectory.__name__).replace('Trajectory', '').lower()
