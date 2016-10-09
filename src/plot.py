#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from constants import STEPS, DELTA_T
from position import Position


def plot_point(pose):
    position = Position.get_position_from_pose(pose)
    x.append(position.x)
    y.append(position.y)


def animate(i):
    x_plot.clear()
    e_x_plot.clear()
    y_plot.clear()
    e_y_plot.clear()

    ts = np.array(t[:len(x)])
    xs = np.array(x)
    xs_ref = np.array(x_ref[:len(x)])
    ys = np.array(y)
    ys_ref = np.array(y_ref[:len(y)])

    zeros = np.array([0 for _ in range(len(x))])
    exs = np.array([b_i - a_i for a_i, b_i in zip(x_ref[:len(x)], x)])
    eys = np.array([b_i - a_i for a_i, b_i in zip(y_ref[:len(y)], y)])

    x_plot.plot(ts, xs_ref, 'r--', ts, xs, 'b')
    e_x_plot.plot(ts, zeros, 'r--', ts, exs, 'b')
    y_plot.plot(ts, ys_ref, 'r--', ts, ys, 'b')
    e_y_plot.plot(ts, zeros, 'r--', ts, eys, 'b')


if __name__ == '__main__':
    t = [i * DELTA_T for i in range(STEPS)]
    x = []
    y = []
    x_ref = [Position.get_position_at(i * DELTA_T).x for i in range(STEPS)]
    y_ref = [Position.get_position_at(i * DELTA_T).y for i in range(STEPS)]

    rospy.init_node('plot')
    subscriber = rospy.Subscriber('plot_data', Pose, plot_point)
    fig, ((x_plot, e_x_plot), (y_plot, e_y_plot)) = plt.subplots(2, 2, sharex=True, sharey=True)
    ani = animation.FuncAnimation(fig, animate, interval=250)
    plt.show()

    rospy.spin()
