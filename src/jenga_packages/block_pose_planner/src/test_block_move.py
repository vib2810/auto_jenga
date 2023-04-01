#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from frankapy import FrankaArm

# 75mm length, 15mm height

class GripperPoseCalculatorNode(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('gripper_pose_calculator')

        # Subscribe to pose topic
        self.pose_sub = rospy.Subscriber('pose_topic', Pose, self.pose_callback)

        # Initialize FrankaArm object
        self.arm = FrankaArm()

        # Initialize variables
        self.gripper_pose = None

        # Spin node
        rospy.spin()

    def pose_callback(self, msg):
        # Convert pose message to a transformation matrix
        pose = tf.transformations.translation_matrix((msg.position.x, msg.position.y, msg.position.z))
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        rotation = tf.transformations.quaternion_matrix(quaternion)
        transform = rotation.dot(pose)

        # Define gripper pose offset from end effector
        offset = tf.transformations.translation_matrix((0, 0, -0.135))

        # Calculate gripper pose
        self.gripper_pose = transform.dot(offset)

        # Move to start pose
        self.arm.goto_pose(tool_pose = self.gripper_pose.flatten().tolist(), duration=1, dynamic=True)
        rospy.sleep(1.5)
        
        # Close gripper to grip block
        self.arm.close_gripper()
        
        # rospy.sleep(0.5)  # Wait for gripper to close

        # Open gripper to release block
        # self.arm.set_gripper_width(gripper_width)
        # rospy.sleep(0.5)  # Wait for gripper to open
        
if __name__ == '__main__':
    GripperPoseCalculatorNode()
