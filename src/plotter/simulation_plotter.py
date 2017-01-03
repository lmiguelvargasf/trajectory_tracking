#!/usr/bin/env python
# coding=utf-8
import datetime
import matplotlib.pyplot as plt
import sqlite3

from .plotter import Plotter, PlotData, get_error


class SimulationPlotter(Plotter):
    def __init__(self, trajectory, steps, delta, controller_name, path):
        Plotter.__init__(self, steps)
        self.delta = delta
        self.controller = controller_name
        self.path = path
        self.plot_data = PlotData()

        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 2, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(1, 2)

    def add_data(self, t, pose, reference):
        self.plot_data.t.append(t)
        self.plot_data.x.append(pose.position.x)
        self.plot_data.y.append(pose.position.y)
        self.plot_data.x_ref.append(reference.x)
        self.plot_data.y_ref.append(reference.y)

    def plot_results(self):
        e_x = get_error(self.plot_data.x_ref, self.plot_data.x)
        e_y = get_error(self.plot_data.y_ref, self.plot_data.y)
        self.plots_part_0[0, 0].plot(self.plot_data.t, self.plot_data.x_ref, 'r--', label=r'$x_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_0[0, 0].plot(self.plot_data.t, self.plot_data.x, 'b', label=r'$x$')
        self.plots_part_0[0, 1].plot(self.plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_0[0, 1].plot(self.plot_data.t, e_x, 'b', label=r'$x_{error}$')
        self.plots_part_0[1, 0].plot(self.plot_data.t, self.plot_data.y_ref, 'r--', label=r'$y_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_0[1, 0].plot(self.plot_data.t, self.plot_data.y, 'b', label=r'$y$')
        self.plots_part_0[1, 1].plot(self.plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_0[1, 1].plot(self.plot_data.t, e_y, 'b', label=r'$y_{error}$')

        e_theta = get_error(self.plot_data.theta_ref, self.plot_data.theta)
        self.plots_part_1[0, 0].plot(self.plot_data.t, self.plot_data.theta_ref, 'r--', label=r'$\theta_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_1[0, 0].plot(self.plot_data.t, self.plot_data.theta, 'b', label=r'$\theta$')
        self.plots_part_1[1, 0].plot(self.plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_1[1, 0].plot(self.plot_data.t, e_theta, 'b', label=r'$\theta_{error}$')

        plt.figure(self.fig_part_1.number)
        trajectory_plot = plt.subplot(122)
        trajectory_plot.plot(self.plot_data.x_ref, self.plot_data.y_ref, 'r--', label=r'${\rm reference}$', lw=self.LINE_WIDTH)
        trajectory_plot.plot(self.plot_data.x, self.plot_data.y, 'b', label=r'${\rm followed}$')

        self.plots_part_2[0].plot(self.plot_data.t, self.plot_data.v_c, 'b', label=r'$v_{c}$')
        self.plots_part_2[1].plot(self.plot_data.t, self.plot_data.w_c, 'b', label=r'$\omega_{c}$')

        self.decorate_plot(self.plots_part_0[0, 0], r'$x\ {\rm and}\ x_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$x[{\rm m}]$')
        self.decorate_plot(self.plots_part_0[0, 1], r'$x_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$x_{error}[{\rm m}]$')
        self.decorate_plot(self.plots_part_0[1, 0], r'$y\ {\rm and}\ y_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$y[{\rm m}]$')
        self.decorate_plot(self.plots_part_0[1, 1], r'$y_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$y_{error}[{\rm m}]$')

        self.decorate_plot(self.plots_part_1[0, 0], r'$\theta,\ \theta_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\theta[{\rm rad}]$')
        self.decorate_plot(self.plots_part_1[1, 0], r'$\theta_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\theta_{error}[{\rm rad}]$')
        self.decorate_plot(trajectory_plot, r'${\rm followed\ trajectory\ vs\ reference\ trajectory}$',
                           r'$x[{\rm m}]$', r'$y[{\rm m}]$')

        self.decorate_plot(self.plots_part_2[0], r'$v_{c}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$v_{c}[{\rm m/s}]$')
        self.decorate_plot(self.plots_part_2[1], r'$\omega_{c}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\omega_{c}[{\rm rad/s}]$')

        title = ''
        if self.controller == 'euler':
            title = r'${\rm Euler\ method\ controller}\ $'
        elif self.controller == 'pid':
            title = r'${\rm PID\ controller}\ $'

        self.fig_part_0.suptitle(title + r'${\rm results - }\ x\ {\rm and}\ y$', fontsize=self.FIGURE_TITLE_SIZE)
        self.fig_part_2.suptitle(title + r'${\rm results - }\ v_{c}\ {\rm and}\ \omega_{c}$',
                                 fontsize=self.FIGURE_TITLE_SIZE)
        self.fig_part_1.suptitle(title + r'${\rm results - }\ \theta\ {\rm and\ trajectory}$',
                                 fontsize=self.FIGURE_TITLE_SIZE)

        plt.show()

    def export_results(self, database_path):
        connection = sqlite3.connect(database_path)
        cursor = connection.cursor()

        table_name = ('_'.join(['euler', 'linear', datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')]))

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS {} (
            t REAL  NOT NULL PRIMARY KEY,
            x REAL NOT NULL,
            x_ref REAL NOT NULL,
            y REAL NOT NULL,
            y_ref REAL NOT NULL,
            theta REAL NOT NULL,
            theta_ref REAL NOT NULL,
            v_c REAL NOT NULL,
            w_c REAL NOT NULL
            )
            """.format(table_name))

        for i in range(len(self.plot_data.t)):
            cursor.execute(
                """
                INSERT INTO {} (t, x, x_ref, y, y_ref, theta, theta_ref, v_c, w_c)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """.format(table_name),
                (self.plot_data.t[i], self.plot_data.x[i], self.plot_data.x_ref[i],
                 self.plot_data.y[i], self.plot_data.y_ref[i], self.plot_data.theta[i],
                 self.plot_data.theta_ref[i], self.plot_data.v_c[i], self.plot_data.w_c[i])
            )
            connection.commit()

        cursor.close()
        connection.close()
