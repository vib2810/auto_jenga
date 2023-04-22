#!/usr/bin/env python

import rospy
# import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from block_pose_planner.srv import *
from frankapy import FrankaArm
import quaternion
from autolab_core import RigidTransform

# 75mm length, 15mm height

def pickup_block(req_pose):
    global fa
    np_quat = quaternion.as_quat_array(np.array([req_pose.pose.orientation.w, req_pose.pose.orientation.x, req_pose.pose.orientation.y, req_pose.pose.orientation.z]))
    transl = np.array([req_pose.pose.position.x, req_pose.pose.position.y, req_pose.pose.position.z + 0.1])
    local_transl = np.array([req_pose.pose.position.x, req_pose.pose.position.y, req_pose.pose.position.z])
    pose_to_set = RigidTransform(rotation=quaternion.as_rotation_matrix(np_quat), translation = transl, from_frame="franka_tool", to_frame="world")
    fa.goto_pose(pose_to_set, use_impedance=False, duration=7)
    local_pose_to_set = RigidTransform(rotation=quaternion.as_rotation_matrix(np_quat), translation = local_transl, from_frame="franka_tool", to_frame="world")
    fa.goto_pose(local_pose_to_set, use_impedance=False, duration=5)
    fa.close_gripper()
    fa.reset_pose(duration=10)

    return True

def drop_block(block_id):
    global fa

    # Hard coded because staging area - need to update when testing with actual limits
    middle_block_pose ={'orientation': np.array([0.020, 0.9985, 0.041, 0.0288]), 
                        'translation': np.array([0.487, -0.08, 0.107])}
    
    drop_pose_quat = quaternion.as_quat_array(middle_block_pose['orientation'])
    drop_pose_transl = middle_block_pose['translation'] + (1 - block_id%3)*np.array([0.025, 0, 0])
    pose_to_drop = RigidTransform(rotation=quaternion.as_rotation_matrix(drop_pose_quat), translation = drop_pose_transl, from_frame="franka_tool", to_frame="world")
    fa.goto_pose(pose_to_drop, use_impedance=False, duration=15)
    drop_pose_local_transl = middle_block_pose['translation'] + (1 - block_id%3)*np.array([0.025, 0, 0]) - np.array([0, 0, 0.1])
    pose_to_drop_local = RigidTransform(rotation=quaternion.as_rotation_matrix(drop_pose_quat), translation = drop_pose_local_transl, from_frame="franka_tool", to_frame="world")
    fa.goto_pose(pose_to_drop_local, use_impedance=False, duration=10)
    fa.open_gripper()
    fa.reset_pose(duration=15)

    return True
    


def recv_pose_callback(req):
    global fa
    print("Received pose information")
    req_pose = req.des_pose
    pickup_done = pickup_block(req_pose)
    if(pickup_done):
        drop_block(req.id)

    return block_poseResponse(True)

def GoToBlockServer():
    s = rospy.Service('go_to_block', block_pose, recv_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    global fa

    rospy.init_node('block_pose_planner')
    fa = FrankaArm(init_node=False)
    GoToBlockServer()
    
    # des_pose = RigidTransform(rotation=np.eye(3), translation=np.array([0.4, 0, 0.165]), from_frame="franka_tool", to_frame="world")
    # current_pose = fa.get_pose()
    # des_pose = RigidTransform(rotation = np.array([[ 0.99735518,  0.00172061 , 0.07252959], [-0.00637742, -0.99375986,  0.11127286], [ 0.07226846, -0.11144111, -0.99113965]]),
    #                         #   translation= np.array([ 0.45871361 ,-0.03142103,  0.0412673 ]), 
    #                           from_frame='franka_tool', to_frame='world')
    # print(current_pose)
    # fa.goto_pose(des_pose, use_impedance=False, duration = 10)
    
