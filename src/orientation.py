#!/usr/bin/env python
import tf.transformations


class Orientation:
    @staticmethod

    def get_orientation_from_pose(pose):
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
            )

        return tf.transformations.euler_from_quaternion(quaternion)
