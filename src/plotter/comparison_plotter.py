#!/usr/bin/env python
import matplotlib.pyplot as plt


class ComparisonPlotter:
    def __init__(self, data_list):
        self.fig_part_0, self.plots_part_0 = plt.subplots(1, 1)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 1, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(2, 1, sharex=True)
        self.fig_part_3, self.plots_part_3 = plt.subplots(2, 1, sharex=True)

    def plot_comparison(self):
        pass
