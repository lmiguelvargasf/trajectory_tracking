#!/usr/bin/env python
from math import atan2, sin, pi
from math import cos

from constants import K_X, DELTA_T, K_THETA, SIMULATION_TIME_IN_SECONDS
from constants import K_Y
from orientation import get_angle_between_0_and_2_pi
from .controller import Controller


class EulerMethodController(Controller):
    def __init__(self, trajectory):
        Controller.__init__(self, trajectory)
        self.theta_ez_n_minus_1 = 0
        self.theta_n_minus_1 = 0

    def set_next_reference(self, reference):
        self.x_ref_n_plus_1 = reference.x
        self.y_ref_n_plus_1 = reference.y

    def get_delta_x_n(self):
        return self.x_ref_n_plus_1 - K_X * (self.x_ref_n - self.x_n) - self.x_n

    def get_delta_y_n(self):
        return self.y_ref_n_plus_1 - K_Y * (self.y_ref_n - self.y_n) - self.y_n

    def get_delta_theta_n(self):
        self.theta_ez_n = get_angle_between_0_and_2_pi(self.theta_ez_n)
        self.theta_n = get_angle_between_0_and_2_pi(self.theta_n)
        self.theta_ez_n_minus_1 = get_angle_between_0_and_2_pi(self.theta_ez_n_minus_1)

        epsilon = 4 * DELTA_T

        alpha = self.theta_ez_n - K_THETA * (self.theta_ez_n_minus_1 - self.theta_n) - self.theta_n

        if alpha == 2 * pi:
            alpha = 0

        if self.trajectory.get_name() == 'astroid' and (
                            abs(SIMULATION_TIME_IN_SECONDS / 4 - self.i * DELTA_T) < epsilon or \
                                abs(SIMULATION_TIME_IN_SECONDS / 2 - self.i * DELTA_T) < epsilon or \
                            abs(3 * SIMULATION_TIME_IN_SECONDS / 4 - self.i * DELTA_T) < epsilon):
            alpha = abs(alpha) * 2

        return alpha

    def compute_theta_ez_n(self):
         return atan2(self.get_delta_y_n(), self.get_delta_x_n())

    def compute_v_c_n(self):
        delta_x_n = self.get_delta_x_n()
        delta_y_n = self.get_delta_y_n()

        return (delta_x_n * cos(self.theta_ez_n) + delta_y_n * sin(self.theta_ez_n)) / DELTA_T

    def compute_w_c_n(self):
        w_n = self.get_delta_theta_n() / DELTA_T

        self.theta_ez_n_minus_1 = self.theta_ez_n
        self.theta_n_minus_1 = self.theta_n
        self.theta_ref_n = self.theta_ez_n

        if self.trajectory.get_name() in ('circular', 'squared', 'astroid'):
            if not 0 <= self.i * DELTA_T < SIMULATION_TIME_IN_SECONDS / 4 + 10 * DELTA_T:
                self.theta_ref_n = get_angle_between_0_and_2_pi(self.theta_ez_n)
                self.theta_n = get_angle_between_0_and_2_pi(self.theta_n)

        # limit angular velocity to be between -pi and pi rad/s
        return atan2(sin(w_n), cos(w_n))

    def compute_control_actions(self, pose, twist, i):
        self.i = i
        self.set_current_orientation(pose.orientation)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at(i * DELTA_T))
        self.set_next_reference(self.trajectory.get_position_at((i + 1) * DELTA_T))

        self.theta_ez_n = self.compute_theta_ez_n()
        self.v_c_n = self.compute_v_c_n()
        self.w_c_n =  self.compute_w_c_n()
