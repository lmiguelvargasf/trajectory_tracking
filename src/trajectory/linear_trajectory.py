#!/usr/bin/env python
from .trajectory import Trajectory


class LinearTrajectory(object, Trajectory):
    def __init__(self, v_x, x_0=0, v_y=0, y_0=0):
        Trajectory.__init__(self)
        self.v_x = v_x
        self.v_y = v_y
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        super(LinearTrajectory, self).get_position_at(t)

        self.position.x = self.v_x * t + self.x_0
        self.position.y = self.v_y * t + self.y_0

        return self.position

    def get_name(self):
        return str(LinearTrajectory.__name__).replace('Trajectory', '').lower()
