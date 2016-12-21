#!/usr/bin/env python
from math import sqrt, cos, pi, sin

from .trajectory import Trajectory


class EpitrochoidTrajectory(object, Trajectory):
    def __init__(self, R, r, d, period, scale_factor=1):
        Trajectory.__init__(self)
        self.R = R
        self.r = r
        self.d = d
        self.period = period
        self.scale_factor = scale_factor

    def get_position_at(self, t):
        super(EpitrochoidTrajectory, self).get_position_at(t)
        self.position.x = self.scale_factor * ((self.R + self.r) * cos(2 * pi * t/ self.period) - self.d * cos(((self.R + self.r) / self.r) * 2 * pi * t / self.period))
        self.position.y = self.scale_factor * ((self.R + self.r) * sin(2 * pi * t/ self.period) - self.d * sin(((self.R + self.r) / self.r) * 2 * pi * t / self.period))

        return self.position

    def get_name(self):
        return str(EpitrochoidTrajectory.__name__).replace('Trajectory', '').lower()
