#!/usr/bin/env python
import rospy
from trajectory_tracking.srv import TrajectoryPoint, TrajectoryPointResponse
from geometry_msgs.msg import Point

def compute_position(request):
    position = Point()
    position.x = 0.0
    position.y = 1.0
    position.z = 2.0
    return position


if __name__ == '__main__':
    rospy.init_node('trajectory_server')
    service = rospy.Service('trajectory', TrajectoryPoint, compute_position)
    rospy.spin()
