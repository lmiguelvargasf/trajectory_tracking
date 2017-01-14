#!/usr/bin/env python
from math import atan2, sin, cos

from .controller import Controller


class PIDController(Controller):
    def __init__(self, trajectory, simulation_data, control_constants, speed_limits):
        Controller.__init__(self, trajectory, simulation_data)
        self.K_P_V = control_constants['kpv']
        self.K_I_V = control_constants['kiv']
        self.K_D_V = control_constants['kdv']

        self.K_P_W = control_constants['kpw']
        self.K_I_W = control_constants['kiw']
        self.K_D_W = control_constants['kdw']

        self.v_c_nm1 = 0
        self.w_c_nm1 = 0

        self.e_v_nm1 = 0
        self.e_w_nm1 = 0
        self.e_v_nm2 = 0
        self.e_w_nm2 = 0

        self.MAX_V = speed_limits['linear']
        self.MAX_W = speed_limits['angular']

    def set_next_reference(self):
        reference = self.trajectory.get_position_at((self.i + 1) * self.delta)
        self.x_ref_n_plus_1 = reference.x
        self.y_ref_n_plus_1 = reference.y

    def compute_control_actions(self, pose, twist, i):
        self.i = i
        self.set_current_orientation(pose.orientation)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at(i * self.delta))
        self.set_next_reference()

        v, w = self.get_velocities(twist)

        v_ref_n = self.compute_linear_speed_reference()
        v_ref_n = self.limit_linear_speed_reference(v_ref_n)

        w_ref_n = self.compute_angular_speed_reference()
        w_ref_n = self.limit_angular_speed_reference(w_ref_n)

        self.e_v_n, self.e_w_n = self.compute_errors(v, v_ref_n, w, w_ref_n)

        a_v, b_v, c_v = self.compute_v_pid_factors()
        a_w, b_w, c_w = self.compute_w_pid_factors()

        self.v_c_n = self.v_c_nm1 + a_v * self.e_v_n + b_v * self.e_v_nm1 + c_v * self.e_v_nm2
        self.w_c_n = self.w_c_nm1 + a_w * self.e_w_n + b_w * self.e_w_nm1 + c_w * self.e_w_nm2

        self.limit_linear_speed_control_action()
        self.limit_angular_speed_control_action()
        self.store_values_for_next_iteration()

    def store_values_for_next_iteration(self):
        self.v_c_nm1 = self.v_c_n
        self.w_c_nm1 = self.w_c_n
        self.e_v_nm2 = self.e_v_nm1
        self.e_w_nm2 = self.e_w_nm1
        self.e_v_nm1 = self.e_v_n
        self.e_w_nm1 = self.e_w_n

    def limit_angular_speed_control_action(self):
        if self.w_c_n > self.MAX_W:
            self.w_c_n = self.MAX_W
        elif self.w_c_n < -self.MAX_W:
            self.w_c_n = -self.MAX_W

    def limit_linear_speed_control_action(self):
        if self.v_c_n > self.MAX_V:
            self.v_c_n = self.MAX_V
        elif self.v_c_n < -self.MAX_V:
            self.v_c_n = -self.MAX_V

    def compute_errors(self, v, v_ref_n, w, w_ref_n):
        e_v = v_ref_n - v
        e_w = w_ref_n - w
        return e_v, e_w

    def compute_w_pid_factors(self):
        a_w = self.K_P_W + self.K_I_W * self.delta / 2 + self.K_D_W / self.delta
        b_w = - self.K_P_W + self.K_I_W * self.delta / 2 - 2 * self.K_D_W / self.delta
        c_w = self.K_D_W / self.delta
        return a_w, b_w, c_w

    def compute_v_pid_factors(self):
        a_v = self.K_P_V + self.K_I_V * self.delta / 2 + self.K_D_V / self.delta
        b_v = -self.K_P_V + self.K_I_V * self.delta / 2 - 2 * self.K_D_V / self.delta
        c_v = self.K_D_V / self.delta
        return a_v, b_v, c_v

    def limit_angular_speed_reference(self, w_ref_n):
        if w_ref_n > self.MAX_W:
            w_ref_n = self.MAX_W
        elif w_ref_n < -self.MAX_W:
            w_ref_n = -self.MAX_W
        return w_ref_n

    def compute_angular_speed_reference(self):
        self.theta_ref_n = atan2(self.y_ref_n_plus_1 - self.y_n, self.x_ref_n_plus_1 - self.x_n)
        self.theta_n = atan2(sin(self.theta_n), cos(self.theta_n))
        w_ref_n = (self.theta_ref_n - self.theta_n) / self.delta

        return w_ref_n

    def limit_linear_speed_reference(self, v_ref_n):
        if v_ref_n > self.MAX_V:
            v_ref_n = self.MAX_V
        elif v_ref_n < -self.MAX_V:
            v_ref_n = -self.MAX_V
        return v_ref_n

    def compute_linear_speed_reference(self):
        v_x_ref = (self.x_ref_n_plus_1 - self.x_n) / self.delta
        v_y_ref = (self.y_ref_n_plus_1 - self.y_n) / self.delta
        v_ref_n = (v_x_ref ** 2 + v_y_ref ** 2) ** 0.5
        return v_ref_n

    def get_velocities(self, twist):
        v_x = twist.linear.x
        v_y = twist.linear.y
        v = (v_x ** 2 + v_y ** 2) ** 0.5
        w = twist.angular.z

        return v, w
