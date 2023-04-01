#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

def pose_publisher():
    # Initialize node
    rospy.init_node('dummy_pose_publisher')

    # Create publisher for pose topic
    pose_pub = rospy.Publisher('pose_topic', Pose, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(10) # 10 Hz

    # Create a pose message
    pose_msg = Pose()

    # Set the position and orientation of the pose message
    pose_msg.position.x = 0.1
    pose_msg.position.y = 0.2
    pose_msg.position.z = 0.3
    pose_msg.orientation.x = 0.0
    pose_msg.orientation.y = 0.0
    pose_msg.orientation.z = 0.0
    pose_msg.orientation.w = 1.0

    # Publish the pose message
    while not rospy.is_shutdown():
        pose_pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass

