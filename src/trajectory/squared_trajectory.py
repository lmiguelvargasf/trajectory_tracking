#!/usr/bin/env python
from .trajectory import Trajectory


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

    def get_name(self):
        return str(SquaredTrajectory.__name__).replace('Trajectory', '').lower()
