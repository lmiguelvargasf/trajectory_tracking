#!/usr/bin/env python
from math import sqrt, cos, pi, sin

from .trajectory import Trajectory


class LemniscateTrajectory(object, Trajectory):
    def __init__(self, radius, period):
        Trajectory.__init__(self)
        self.radius = radius
        self. period = period

    def get_position_at(self, t):
        super(LemniscateTrajectory, self).get_position_at(t)
        self.position.x = 2 * cos(2 * pi* t / self.period) / (sin(2 * pi * t / self.period) ** 2 + 1)
        self.position.y = 2 * sin(2 * pi* t / self.period) * cos(2 * pi* t / self.period) / (sin(2 * pi * t / self.period) ** 2 + 1)

        return self.position

    def get_name(self):
        return str(LemniscateTrajectory.__name__).replace('Trajectory', '').lower()