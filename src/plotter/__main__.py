import sqlite3

import sys

from .simulation_plotter import SimulationPlotter
from .plotter import PlotData
from .constants import QUERIES

if __name__ == '__main__':
    print("Plotting results of the last simulation...")
    plot_data = PlotData()

    connection = sqlite3.connect(sys.argv[1:][0])
    cursor = connection.cursor()

    cursor.execute(QUERIES['select_sim'])

    simulation_name = cursor.fetchone()[0]
    controller = str(simulation_name).split('_')[0]

    cursor.execute(QUERIES['select_data'].format(simulation_name))

    for row in cursor.fetchall():
        plot_data.t.append(row[0])
        plot_data.x.append(row[1])
        plot_data.x_ref.append(row[2])
        plot_data.y.append(row[3])
        plot_data.y_ref.append(row[4])
        plot_data.theta.append(row[5])
        plot_data.theta_ref.append(row[6])
        plot_data.v_c.append(row[7])
        plot_data.w_c.append(row[8])
        plot_data.zeros.append(0)

    plotter = SimulationPlotter(plot_data, controller)
    plotter.plot_results()

    cursor.close()
    connection.close()
