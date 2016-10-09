#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from constants import STEPS, DELTA_T
from position import Position

t = [i * DELTA_T for i in range(STEPS)]
x = []
y = []
x_ref = [Position.get_position_at(i * DELTA_T).x for i in range(STEPS)]
y_ref = [Position.get_position_at(i * DELTA_T).y for i in range(STEPS)]


def plot_point(pose):
    position = Position.get_position_from_pose(pose)
    x.append(position.x)
    y.append(position.y)


if __name__ == '__main__':
    rospy.init_node('plot')
    subscriber = rospy.Subscriber('plot_data', Pose, plot_point)
    rospy.spin()

fig, ((x_plot, e_x_plot), (y_plot, e_y_plot)) = plt.subplots(2, 2, sharex=True, sharey=True)

def animate(i):
    x_plot.clear()
    e_x_plot.clear()
    y_plot.clear()
    e_y_plot.clear()
    x_plot.plot(t[:len(x)], x_ref[:len(x)], 'r--', t[:len(x)], x, 'b')
    e_x_plot.plot(t[:len(x)], [0 for _ in range(len(x))], 'r--',
                  t[:len(x)], [b_i - a_i for a_i, b_i in zip(x_ref[:len(x)], x)], 'b')
    y_plot.plot(t[:len(y)], y_ref[:len(y)], 'r--', t[:len(y)], y, 'b')
    e_y_plot.plot(t[:len(y)], [0 for _ in range(len(y))], 'r--',
                  t[:len(y)], [b_i - a_i for a_i, b_i in zip(y_ref[:len(y)], y)], 'b')

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()
