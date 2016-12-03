#!/usr/bin/env python
import unittest

from trajectory.trajectory import NegativeTimeException, Trajectory


class TrajectoryTest(unittest.TestCase):

    def test_when_time_is_negative_then_an_exception_is_raised(self):
        trajectory = Trajectory()
        self.assertRaises(NegativeTimeException, trajectory.get_position_at, -1)
