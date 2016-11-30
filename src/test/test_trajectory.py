#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory import LinearTrajectory, NegativeTimeException, CircularTrajectory, Trajectory, SquaredTrajectory


class TrajectoryTest(unittest.TestCase):

    def test_when_time_is_negative_then_an_exception_is_raised(self):
        trajectory = Trajectory()
        self.assertRaises(NegativeTimeException, trajectory.get_position_at, -1)



class LinearTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = LinearTrajectory(1, 0, 2, 0)
        self.expected_position = Point()

    def test_given_trajectory_at_origin_when_getting_position_after_1s_then_position_is_returned(self):
        self.expected_position.x = 1
        self.expected_position.y = 2
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(1))

    def test_given_trajectory_not_at_origin_when_getting_position_after_1s_then_position_is_returned(self):
        self.trajectory.x_0 = 1
        self.trajectory.y_0 = 2
        self.expected_position.x = 2
        self.expected_position.y = 4
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(1))


class CircularTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = CircularTrajectory(5, 4)
        self.expected_position = Point()

    def test_given_circular_trajectory_centered_at_origin_when_getting_position_then_position_is_returned(self):
        self.expected_position.x = 5
        self.expected_position.y = 0
        self.trajectory = CircularTrajectory(5, 4)
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(1), 0.01)

    def test_given_circular_trajectory_not_centered_at_origin_when_getting_position_then_position_is_returned(self):
        self.trajectory.x_0 = 5
        self.trajectory.y_0 = 5
        self.expected_position.x = 10
        self.expected_position.y = 5
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(1), 0.01)

    def assertPositionAlmostEqual(self, expected, actual, delta):
        self.assertAlmostEqual(expected.x, actual.x, delta=delta)
        self.assertAlmostEqual(expected.y, actual.y, delta=delta)


class SquaredTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = SquaredTrajectory(3, 4)
        self.expected_position = Point()

    def test_given_squared_trajectory_with_corner_at_the_origin_when_getting_position_then_position_is_returned(self):
        self.expected_position.x = 3
        self.expected_position.y = 3
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(2))

    def test_given_squared_trajectory_with_corner_not_at_the_origin_when_getting_position_then_position_is_returned(self):
        self.trajectory.x_0 = 1
        self.trajectory.y_0 = 1
        self.expected_position.x = 4
        self.expected_position.y = 4
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(2))
