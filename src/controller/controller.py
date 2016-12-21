#!/usr/bin/env python

from orientation import get_euler_orientation
from util.util import create_trajectory


class Controller:
    def __init__(self):
        self.trajectory = create_trajectory()
        self.theta_ref_n = 0

    def set_current_orientation(self, orientation):
        self.theta_n = get_euler_orientation(orientation)[2]

    def set_current_position(self, position):
        self.x_n = position.x
        self.y_n = position.y

    def set_current_reference(self, reference):
        self.x_ref_n = reference.x
        self.y_ref_n = reference.y
