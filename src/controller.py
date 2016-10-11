#!/usr/bin/env python
import math

from constants import K_X, DELTA_T, K_Y, TRAJECTORY, K_THETA, CONTROLLER
from orientation import Orientation
from trajectory import create_trajectory


def create_controller():
    if CONTROLLER == 'euler':
        return EulerMethodController()
    elif CONTROLLER == 'trapezoidal':
        return TrapezoidalRuleController()


class Controller:
    def __init__(self):
        self.trajectory = create_trajectory(TRAJECTORY)
        self.theta_ez_n_minus_1 = 0
        self.theta_n_minus_1 = 0

    def compute_theta_ez_n(self):
        pass

    def set_current_position(self, position):
        self.x_n = position.x
        self.y_n = position.y

    def set_current_reference(self, reference):
        self.x_ref_n = reference.x
        self.y_ref_n = reference.y

    def set_next_reference(self, reference):
        self.x_ref_n_plus_1 = reference.x
        self.y_ref_n_plus_1 = reference.y

class EulerMethodController(Controller):
    def __init__(self):
        Controller.__init__(self)

    def compute_theta_ez_n(self):
        numerator = self.y_ref_n_plus_1 - K_X * (self.y_ref_n - self.y_n) - self.y_n
        denominator = self.x_ref_n_plus_1 - K_Y * (self.x_ref_n - self.x_n) - self.x_n

        self.theta_ez_n = math.atan2(numerator, denominator)

    def compute_v_n(self):
        operand_0 = self.x_ref_n_plus_1 - K_X * (self.x_ref_n - self.x_n) - self.x_n
        operand_1 = self.y_ref_n_plus_1 - K_Y * (self.y_ref_n - self.y_n) - self.y_n

        self.v_n = (operand_0 * math.cos(self.theta_ez_n) + operand_1 * math.sin(self.theta_ez_n)) / DELTA_T

    def compute_w_n(self, current_orientation):
        w_n =(self.theta_ez_n
               - K_THETA * (self.theta_ez_n_minus_1 - self.theta_n_minus_1)
               - self.theta_n_minus_1) \
              / DELTA_T

        self.theta_ez_n_minus_1 = self.theta_ez_n
        self.theta_n_minus_1 = current_orientation[2]

        self.w_n = math.atan2(math.sin(w_n), math.cos(w_n))

    def compute_control_actions(self, pose, i):
        current_orientation = Orientation.get_orientation_from_pose(pose)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at(i * DELTA_T))
        self.set_next_reference(self.trajectory.get_position_at((i + 1) * DELTA_T))

        self.compute_theta_ez_n()
        self.compute_v_n()
        self.compute_w_n(current_orientation)


class TrapezoidalRuleController(Controller):
    def __init__(self):
        Controller.__init__(self)
