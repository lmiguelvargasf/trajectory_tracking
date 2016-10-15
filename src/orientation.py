#!/usr/bin/env python
from math import pi
from tf.transformations import euler_from_quaternion


def get_euler_orientation(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )

    return euler_from_quaternion(quaternion)

def get_angle_between_0_and_2_pi(theta):
    if -pi <= theta < 0:
        return 2 * pi + theta
    return theta
