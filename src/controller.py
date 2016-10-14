#!/usr/bin/env python
from math import sin, cos, atan2, pi, fabs

from constants import K_X, DELTA_T, K_Y, K_THETA, K_P_V, K_I_V, K_D_V, K_P_W, K_I_W, K_D_W, CONTROLLER, MAX_V, MAX_W, \
    TRAJECTORY, SIMULATION_TIME_IN_SECONDS
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
        self.theta_ref_n = 0

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
        self.theta_ref_n = self.theta_ez_n
        self.v_c_n = self.compute_v_c_n()
        self.w_c_n =  self.compute_w_c_n()


def get_angle_between_0_and_2_pi(theta):
    if -pi <= theta < 0:
        return 2 * pi + theta
    return theta

class PIDController(Controller):
    def __init__(self):
        Controller.__init__(self)
        self.K_P_V = K_P_V
        self.K_I_V = K_I_V
        self.K_D_V = K_D_V

        self.K_P_W = K_P_W
        self.K_I_W = K_I_W
        self.K_D_W = K_D_W

        self.v_c_nm1 = 0
        self.w_c_nm1 = 0

        self.e_v_nm1 = 0
        self.e_w_nm1 = 0
        self.e_v_nm2 = 0
        self.e_w_nm2 = 0

        self.MAX_V = MAX_V
        self.MAX_W = MAX_W

    def compute_control_actions(self, pose, twist, i):
        self.i = i
        self.set_current_orientation(pose.orientation)
        self.set_current_position(pose.position)
        self.set_current_reference(self.trajectory.get_position_at((i + 1) * DELTA_T))

        w = self.get_angular_speed(twist)
        v = self.get_linear_speed(twist)

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
        self.e_w_n = atan2(sin(e_w), cos(e_w))
        return e_v, e_w

    def compute_w_pid_factors(self):
        a_w = self.K_P_W + self.K_I_W * DELTA_T / 2 + self.K_D_W / DELTA_T
        b_w = - self.K_P_W + self.K_I_W * DELTA_T / 2 - 2 * self.K_D_W / DELTA_T
        c_w = self.K_D_W / DELTA_T
        return a_w, b_w, c_w

    def compute_v_pid_factors(self):
        a_v = self.K_P_V + self.K_I_V * DELTA_T / 2 + self.K_D_V / DELTA_T
        b_v = -self.K_P_V + self.K_I_V * DELTA_T / 2 - 2 * self.K_D_V / DELTA_T
        c_v = self.K_D_V / DELTA_T
        return a_v, b_v, c_v

    def limit_angular_speed_reference(self, w_ref_n):
        if w_ref_n > self.MAX_W:
            w_ref_n = self.MAX_W
        elif w_ref_n < -self.MAX_W:
            w_ref_n = -self.MAX_W
        return w_ref_n

    def compute_angular_speed_reference(self):
        theta_ref = atan2(self.y_ref_n - self.y_n, self.x_ref_n - self.x_n)
        self.theta_ref_n = theta_ref

        w_ref_n = (theta_ref - self.theta_n) / DELTA_T


        if TRAJECTORY == 'circular':
            if -pi < theta_ref < -pi / 2  and pi / 2 < self.theta_n < pi:
                self.theta_ref_n = -theta_ref
                w_ref_n = (2 * pi + theta_ref - self.theta_n) / DELTA_T
        elif TRAJECTORY == 'squared':
            if pi / 4 < self.theta_n < pi and -pi < theta_ref < -pi / 2:
                w_ref_n = (2 * pi + theta_ref - self.theta_n) / DELTA_T
            elif -pi < self.theta_n < -pi / 2 and pi / 2 < theta_ref < pi:
                w_ref_n = (2 * pi - theta_ref + self.theta_n) / DELTA_T
            elif - pi / 2 < self.theta_n < 0 and -pi < theta_ref < -pi /2:
                w_ref_n = (self.theta_n - theta_ref) / DELTA_T
            elif (self.theta_n == pi or self.theta_n == -pi) and (theta_ref == pi or theta_ref == -pi):
                w_ref_n = 0

        return w_ref_n

    def limit_linear_speed_reference(self, v_ref_n):
        if v_ref_n > self.MAX_V:
            v_ref_n = self.MAX_V
        elif v_ref_n < -self.MAX_V:
            v_ref_n = -self.MAX_V
        return v_ref_n

    def compute_linear_speed_reference(self):
        v_x_ref = (self.x_ref_n - self.x_n) / DELTA_T
        v_y_ref = (self.y_ref_n - self.y_n) / DELTA_T
        v_ref_n = (v_x_ref ** 2 + v_y_ref ** 2) ** 0.5
        return v_ref_n

    def get_linear_speed(self, twist):
        v_x = twist.linear.x
        v_y = twist.linear.y
        v = (v_x ** 2 + v_y ** 2) ** 0.5
        return v

    def get_angular_speed(self, twist):
        return twist.angular.z
