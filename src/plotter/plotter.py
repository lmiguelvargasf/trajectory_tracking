#!/usr/bin/env python
# coding=utf-8
import matplotlib.pyplot as plt
from .constants import ARRAY_NAMES


def get_error(reference, actual):
    return [(b_i - a_i) for b_i , a_i in zip(reference, actual)]


def get_data_container(controller_name):
    data = {array: [] for array in ARRAY_NAMES}
    data['controller_name'] = controller_name
    return data


class Plotter:
    def __init__(self):
        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 2, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(1, 2)
