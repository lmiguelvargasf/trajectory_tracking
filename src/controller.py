#!/usr/bin/env python
from math import sin, cos, atan2

from constants import K_X, DELTA_T, K_Y, K_THETA, K_P_V, K_I_V, K_D_V, K_P_W, K_I_W, K_D_W, CONTROLLER
from orientation import get_euler_orientation
from trajectory import create_trajectory


def create_controller():
    if CONTROLLER == 'euler':
        return EulerMethodController()
    elif CONTROLLER == 'pid':
        return PIDController()


class Controller:
    def __init__(self):
        self.trajectory = create_trajectory()

    def set_current_orientation(self, orientation):
        self.theta_n = get_euler_orientation(orientation)[2]

    def set_current_position(self, position):
        self.x_n = position.x
        self.y_n = position.y

    def set_current_reference(self, reference):
        self.x_ref_n = reference.x
        self.y_ref_n = reference.y


class EulerMethodController(Controller):
    def __init__(self):
        Controller.__init__(self)
        self.trajectory = create_trajectory()
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
        return self.theta_ez_n - K_THETA * (self.theta_ez_n_minus_1 - self.theta_n_minus_1) - self.theta_n_minus_1

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

        # limit angular velocity to be between -pi and pi rad/s
        return atan2(sin(w_n), cos(w_n))

    def compute_control_actions(self, pose, twist, i):
        self.set_current_orientation(pose.orientation)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at(i * DELTA_T))
        self.set_next_reference(self.trajectory.get_position_at((i + 1) * DELTA_T))

        self.theta_ez_n = self.compute_theta_ez_n()
        self.v_c_n = self.compute_v_c_n()
        self.w_c_n =  self.compute_w_c_n()


class PIDController(Controller):
    def __init__(self):
        Controller.__init__(self)

