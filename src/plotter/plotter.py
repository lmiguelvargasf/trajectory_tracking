#!/usr/bin/env python
# coding=utf-8
from .constants import ARRAY_NAMES, COLORS, PLOT


def get_error(reference, actual):
    return [(b_i - a_i) for b_i , a_i in zip(reference, actual)]


def get_data_container(controller_name):
    data = {array: [] for array in ARRAY_NAMES}
    data['controller_name'] = controller_name
    return data


class Plotter:
    def __init__(self, t, zeros):
        self.t = t
        self.zeros = zeros

    def plot_zeros(self, plot):
        plot.plot(self.t, self.zeros, COLORS['ref'], label=r'$e=0$', lw=PLOT['line_width'])

    def plot_actual_data(self, plot, tag, ys, xs=None, color=COLORS['line_0']):
        if xs is None:
            xs = self.t
        label= r'$' + tag + r'$'
        plot.plot(xs, ys, color, label=label)

    def plot_reference(self, plot, tag, ys, xs=None):
        if xs is None:
            xs = self.t

        label = r'$' + tag + (r'_{ref}$' if tag in ('x', 'y', r'\theta') else r'$')
        plot.plot(xs, ys, COLORS['ref'], label=label, lw=PLOT['line_width'])

    def _decorate_plot(self, plot, title, x_label, y_label):
        plot.set_title(title, fontsize=PLOT['plot_title_size'])
        plot.set_xlabel(x_label, fontsize=PLOT['axis_label_size'])
        plot.set_ylabel(y_label, fontsize=PLOT['axis_label_size'])
        plot.legend(loc=0)
        plot.grid()
