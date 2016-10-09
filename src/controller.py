#!/usr/bin/env python
import math

from constants import K_V, DELTA_T, K_W


class Controller:
    def __init__(self):
        self.theta_ez_n_minus_1 = 0
        self.theta_n_minus_1 = 0

    def get_theta_ez_n(self, next_reference, current_reference, current_position):
        x_ref_n_plus_1 = next_reference.x
        y_ref_n_plus_1 = next_reference.y
        x_ref_n = current_reference.x
        y_ref_n = current_reference.y
        x_n = current_position.x
        y_n = current_position.y

        numerator = y_ref_n_plus_1 - K_V * (y_ref_n - y_n) - y_n
        denominator = x_ref_n_plus_1 - K_V * (x_ref_n - x_n) - x_n

        return math.atan2(numerator, denominator)

    def get_V_n(selfl, next_reference, current_reference, current_position, theta_ez_n):
        x_ref_n_plus_1 = next_reference.x
        y_ref_n_plus_1 = next_reference.y
        x_ref_n = current_reference.x
        y_ref_n = current_reference.y
        x_n = current_position.x
        y_n = current_position.y

        operand_0 = x_ref_n_plus_1 - K_V * (x_ref_n - x_n) - x_n
        operand_1 = y_ref_n_plus_1 - K_V * (y_ref_n - y_n) - y_n

        return (operand_0 * math.cos(theta_ez_n) + operand_1 * math.sin(theta_ez_n)) / DELTA_T

    def get_w_n(self, theta_ez_n, current_orientation):
        w_n = (theta_ez_n - K_W * (self.theta_ez_n_minus_1 - self.theta_n_minus_1) - self.theta_n_minus_1) / DELTA_T

        self.theta_ez_n_minus_1 = theta_ez_n
        self.theta_n_minus_1 = current_orientation[2]

        return math.atan2(math.sin(w_n), math.cos(w_n))
