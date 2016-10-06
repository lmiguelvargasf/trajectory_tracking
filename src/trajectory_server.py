#!/usr/bin/env python
import rospy
from trajectory_tracking.srv import TrajectoryPoint, TrajectoryPointResponse

def compute_trajectory(request):
    pass


if __name__ == '__main__':
    rospy.init_node('trajectory_server')
    service = rospy.Service('trajectory', TrajectoryPoint, compute_trajectory)
    rospy.spin()
