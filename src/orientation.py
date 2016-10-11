#!/usr/bin/env python
import tf.transformations


def get_euler_orientation(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )

    return tf.transformations.euler_from_quaternion(quaternion)
