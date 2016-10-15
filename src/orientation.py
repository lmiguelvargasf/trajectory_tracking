#!/usr/bin/env python
from tf.transformations import euler_from_quaternion


def get_euler_orientation(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )

    return euler_from_quaternion(quaternion)
