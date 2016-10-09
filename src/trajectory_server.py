#!/usr/bin/env python
import rospy
from trajectory_tracking.srv import TrajectoryPoint
from geometry_msgs.msg import Point

def compute_position(request):
    t = request.t

    position = Point()
    position.x = 0.05 * t
    position.y = 0.05 * t
    position.z = 0.0

    return position


if __name__ == '__main__':
    rospy.init_node('trajectory_server')
    service = rospy.Service('trajectory', TrajectoryPoint, compute_position)
    rospy.spin()
