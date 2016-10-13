#!/usr/bin/env python
import matplotlib.pyplot as plt

from constants import STEPS, DELTA_T
from trajectory import create_trajectory


class Plotter:
    def __init__(self):
        trajectory = create_trajectory()
        self.t = [i * DELTA_T for i in range(STEPS)]
        self.x_ref = [trajectory.get_position_at(i * DELTA_T).x for i in range(STEPS)]
        self.y_ref = [trajectory.get_position_at(i * DELTA_T).y for i in range(STEPS)]
        self.x = []
        self.y = []
        self.theta = []
        self.theta_ref = []
        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1 = plt.figure()
        self.plots_part_1 = [
            plt.subplot(122),
            plt.subplot(221),
            plt.subplot(223),
        ]

    def add_point(self, pose):
        self.x.append(pose.position.x)
        self.y.append(pose.position.y)

    def decorate_plot(self, plot, title, x_label, y_label):
        plot.set_title(title)
        plot.legend(loc=0)
        plot.set_xlabel(x_label)
        plot.set_ylabel(y_label)
        plot.grid()

    def plot_results(self):
        zeros = [0 for _ in range(STEPS)]
        e_x = [(b_i - a_i) for a_i , b_i in zip(self.x, self.x_ref)]
        e_y = [(b_i - a_i) for a_i , b_i in zip(self.y, self.y_ref)]
        self.plots_part_0[0, 0].plot(self.t, self.x_ref, 'r--', label='ref', lw=3)
        self.plots_part_0[0, 0].plot(self.t, self.x, 'b', label='real')
        self.plots_part_0[0, 1].plot(self.t, zeros, 'r--', label='e=0', lw=2)
        self.plots_part_0[0, 1].plot(self.t, e_x, 'b', label='x error')
        self.plots_part_0[1, 0].plot(self.t, self.y_ref, 'r--', label='ref', lw=2)
        self.plots_part_0[1, 0].plot(self.t, self.y, 'b', label='real')
        self.plots_part_0[1, 1].plot(self.t, zeros, 'r--', label='e=0', lw=2)
        self.plots_part_0[1, 1].plot(self.t, e_y, 'b', label='y error')

        e_theta = [(b_i - a_i) for a_i , b_i in zip(self.theta, self.theta_ref)]
        self.plots_part_1[0].plot(self.x_ref, self.y_ref, 'r--', label='ref', lw=2)
        self.plots_part_1[0].plot(self.x, self.y, 'b', label='real')
        self.plots_part_1[1].plot(self.t, self.theta_ref, 'r--', label='ref', lw=2)
        self.plots_part_1[1].plot(self.t, self.theta, 'b', label='real')
        self.plots_part_1[2].plot(self.t, zeros, 'r--', label='e=0', lw=2)
        self.plots_part_1[2].plot(self.t, e_theta, 'b', label='th error')

        self.decorate_plot(self.plots_part_0[0, 0], 'x and x ref vs. t', 't[s]', 'x[m]')
        self.decorate_plot(self.plots_part_0[0, 1], 'x error vs. t', 't[s]', 'x-error[m]')
        self.decorate_plot(self.plots_part_0[1, 0], 'y and y ref vs. t', 't[s]', 'y[m]')
        self.decorate_plot(self.plots_part_0[1, 1], 'y error vs. t', 't[s]', 'y-error[m]')

        self.decorate_plot(self.plots_part_1[0], 'followed vs reference trajectory', 'x[m]', 'y[m]')
        self.decorate_plot(self.plots_part_1[1], 'theta and theta-ez vs t', 't[s]', 'theta[rad]')
        self.decorate_plot(self.plots_part_1[2], 'theta error vs t', 't[s]', 'theta-error[rad]')

        plt.show()
