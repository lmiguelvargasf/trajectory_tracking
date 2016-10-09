#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from constants import DELTA_T

current_pose = None

def get_pose(message):
    global current_pose
    
    current_pose = message.pose[2]


if __name__ == '__main__':
    rospy.init_node('pose')
    subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, get_pose)
    publisher = rospy.Publisher('pose_10_hz', Pose, queue_size=1)

    while current_pose is None:
        pass
    
    rate = rospy.Rate(10)
    rate = rospy.Rate(int(1 / DELTA_T))
    while not rospy.is_shutdown():
        publisher.publish(current_pose)
        rate.sleep()
