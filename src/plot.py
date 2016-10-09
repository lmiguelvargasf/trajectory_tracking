#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from wrapt import synchronized

from constants import STEPS, DELTA_T
from position import Position


def plot_point(pose):
    position = Position.get_position_from_pose(pose)
    x.append(position.x)
    y.append(position.y)



@synchronized
def animate(k):
    for k in range(plots.shape[0]):
        for j in range(plots.shape[1]):
            plots[k, j].clear()

    tsx = np.array(t[:len(x)])
    tsy = np.array(t[:len(y)])
    xs = np.array(x)
    xs_ref = np.array(x_ref[:len(x)])
    ys = np.array(y)
    ys_ref = np.array(y_ref[:len(y)])

    zeros_x = np.array([0 for _ in range(len(x))])
    zeros_y = np.array([0 for _ in range(len(y))])
    exs = np.array([b_i - a_i for a_i, b_i in zip(x_ref[:len(x)], x)])
    eys = np.array([b_i - a_i for a_i, b_i in zip(y_ref[:len(y)], y)])

    plots[0, 0].plot(tsx, xs_ref, 'r--')
    plots[0, 0].plot(tsx, xs, 'b')

    plots[0, 1].plot(tsx, zeros_x, 'r--')
    plots[0, 1].plot(tsx, exs, 'b',)

    plots[1, 0].plot(tsy, ys_ref, 'r--')
    plots[1, 0].plot(tsy, ys, 'b',)

    plots[1, 1].plot(tsy, zeros_y, 'r--')
    plots[1, 1].plot(tsy, eys, 'b',)


if __name__ == '__main__':
    t = [i * DELTA_T for i in range(STEPS)]
    x = []
    y = []
    x_ref = [Position.get_position_at(i * DELTA_T).x for i in range(STEPS)]
    y_ref = [Position.get_position_at(i * DELTA_T).y for i in range(STEPS)]

    rospy.init_node('plot')
    subscriber = rospy.Subscriber('plot_data', Pose, plot_point)
    fig, plots = plt.subplots(2, 2, sharex=True, sharey=True)
    ani = animation.FuncAnimation(fig, animate, interval=250)
    plt.show()

    rospy.spin()
