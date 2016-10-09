#!/usr/bin/env python
import math

from constants import K_V, DELTA_T, K_W, TRAJECTORY_TYPE
from orientation import Orientation
from trajectory import create_trajectory

class Controller:
    def __init__(self):
        self.trajectory = create_trajectory(TRAJECTORY_TYPE)
        self.theta_ez_n_minus_1 = 0
        self.theta_n_minus_1 = 0
        self.theta_ez_n = 0

    def get_theta_ez_n(self):
        numerator = self.y_ref_n_plus_1 - K_V * (self.y_ref_n - self.y_n) - self.y_n
        denominator = self.x_ref_n_plus_1 - K_V * (self.x_ref_n - self.x_n) - self.x_n

        return math.atan2(numerator, denominator)

    def compute_v_n(self):
        operand_0 = self.x_ref_n_plus_1 - K_V * (self.x_ref_n - self.x_n) - self.x_n
        operand_1 = self.y_ref_n_plus_1 - K_V * (self.y_ref_n - self.y_n) - self.y_n

        return (operand_0 * math.cos(self.theta_ez_n) + operand_1 * math.sin(self.theta_ez_n)) / DELTA_T

    def compute_w_n(self, current_orientation):
        w_n = (self.theta_ez_n - K_W * (self.theta_ez_n_minus_1 - self.theta_n_minus_1) - self.theta_n_minus_1) / DELTA_T

        self.theta_ez_n_minus_1 = self.theta_ez_n
        self.theta_n_minus_1 = current_orientation[2]

        return math.atan2(math.sin(w_n), math.cos(w_n))

    def set_current_position(self, position):
        self.x_n = position.x
        self.y_n = position.y

    def set_current_reference(self, reference):
        self.x_ref_n = reference.x
        self.y_ref_n = reference.y

    def set_next_reference(self, reference):
        self.x_ref_n_plus_1 = reference.x
        self.y_ref_n_plus_1 = reference.y

    def compute_control_actions(self, pose, i):
        current_orientation = Orientation.get_orientation_from_pose(pose)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at(i * DELTA_T))
        self.set_next_reference(self.trajectory.get_position_at((i + 1) * DELTA_T))

        self.theta_ez_n = self.get_theta_ez_n()
        self.v_n = self.compute_v_n()
        self.w_n = self.compute_w_n(current_orientation)
