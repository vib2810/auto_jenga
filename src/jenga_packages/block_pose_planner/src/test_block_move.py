#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from frankapy import FrankaArm
import quaternion
from autolab_core import RigidTransform

# 75mm length, 15mm height

class GripperPoseCalculatorNode(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('gripper_pose_calculator')

        # Subscribe to pose topic
        self.pose_sub = rospy.Subscriber('pose_topic', Pose, self.pose_callback)

        # Initialize FrankaArm object
        self.arm = FrankaArm()
        self.arm.reset_joints()

        # Initialize variables
        self.gripper_pose = None
        

        # Spin node
        rospy.spin()

    def pose_callback(self, msg):

        # Calculate gripper pose

        q_np = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        rotation_matrix = quaternion.as_rotation_matrix(q_np)
        des_pose = RigidTransform(rotation= rotation_matrix,
            translation=np.array([msg.position.x, msg.position.y, msg.position.z-0.135]),
            from_frame='franka_tool', to_frame='world')
        self.arm.goto_pose(des_pose, dynamic=True, use_impedance=False)

        rospy.sleep(5)

        # Close gripper to grip block
        self.arm.close_gripper()

if __name__ == '__main__':
    GripperPoseCalculatorNode()
