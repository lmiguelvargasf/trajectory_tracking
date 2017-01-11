#!/usr/bin/env python
from math import pi, sin

from .trajectory import Trajectory


class LissajousTrajectory(object, Trajectory):
    def __init__(self, A, B, a, b, period, delta=pi/2):
        Trajectory.__init__(self)
        self.A = A
        self.B = B
        self.a = a
        self.b = b
        self.period = period
        self.delta = delta

    def get_position_at(self, t):
        super(LissajousTrajectory, self).get_position_at(t)
        self.position.x = self.A * sin(2 * pi * t * self.a / self.period + self.delta)
        self.position.y = self.B * sin(2 * pi * t * self.b / self.period)

        return self.position

    def get_name(self):
        return str(LissajousTrajectory.__name__).replace('Trajectory', '').lower()
