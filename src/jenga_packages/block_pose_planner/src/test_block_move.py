#!/usr/bin/env python

import rospy
# import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from block_pose_planner.srv import *
import sys
sys.path.append("/home/ros_ws/src/git_packages/frankapy")
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
    fa.reset_joints()

    return True

def drop_block(block_id, layer_id):
    global fa
    print("Dropping block %d in layer %d" %(block_id, layer_id))

    # Hard coded because staging area - need to update when testing with actual limits
    middle_block_pose_even ={'orientation': np.array([0.0, 1.0, 0.0, 0.0]), 
                        'translation': np.array([0.5, 0, 0.5])}
    middle_block_pose_odd = {'orientation': np.array([0, 0.707, 0.707, 0.0]), 
                        'translation': np.array([0.5, 0, 0.5])}
    
    drop_pose_quat = quaternion.as_quat_array(middle_block_pose_even['orientation'])
    if(layer_id%2 == 0):
        drop_pose_quat_local = quaternion.as_quat_array(middle_block_pose_even['orientation'])
        drop_pose_transl = middle_block_pose_even['translation'] + (1 - block_id%3)*np.array([0.025, 0, 0])
        pose_quat_squeeze = quaternion.as_quat_array(middle_block_pose_odd['orientation'])
    else:
        drop_pose_quat_local = quaternion.as_quat_array(middle_block_pose_odd['orientation'])
        drop_pose_transl = middle_block_pose_odd['translation'] + (1 - block_id%3)*np.array([0, 0.026, 0])
        pose_quat_squeeze = quaternion.as_quat_array(middle_block_pose_even['orientation'])

    pose_to_drop = RigidTransform(rotation=quaternion.as_rotation_matrix(drop_pose_quat), translation = drop_pose_transl, from_frame="franka_tool", to_frame="world")
    
    fa.goto_pose(pose_to_drop, use_impedance=False, duration=5)

    drop_pose_local_transl = drop_pose_transl - np.array([0, 0, 0.5]) + (2*layer_id+1)*np.array([0,0,0.0075])
    pose_to_drop_local = RigidTransform(rotation=quaternion.as_rotation_matrix(drop_pose_quat_local), translation = drop_pose_local_transl, from_frame="franka_tool", to_frame="world")
    print(pose_to_drop_local)
    fa.goto_pose(pose_to_drop_local, use_impedance=False, duration=10)
    fa.open_gripper()
    if(block_id == 2):
        fa.goto_pose(pose_to_drop, use_impedance=False, duration=5)
        pose_squeeze_transl = middle_block_pose_even['translation'] - np.array([0,0,0.5]) + (2*layer_id+1)*np.array([0,0,0.0075])
        pose_to_squeeze = RigidTransform(rotation=quaternion.as_rotation_matrix(pose_quat_squeeze), translation = pose_squeeze_transl, from_frame="franka_tool", to_frame="world")
        fa.goto_pose(pose_to_squeeze, use_impedance=False, duration=10)
        fa.close_gripper()
        fa.open_gripper()

    fa.reset_joints()

    return True
    


def recv_pose_callback(req):
    global fa
    global done_flag

    print("Received pose information")
    req_pose = req.des_pose
    pickup_done = pickup_block(req_pose)
    if(pickup_done):
        drop_block(req.block_id, req.layer_id)

    return block_poseResponse(True)

def GoToBlockServer():
    s = rospy.Service('go_to_block', block_pose, recv_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    global fa

    rospy.init_node('block_pose_planner')
    fa = FrankaArm(init_node=False)
    fa.reset_joints()
    fa.open_gripper()
    GoToBlockServer()
    
    # des_pose = RigidTransform(rotation=np.eye(3), translation=np.array([0.4, 0, 0.165]), from_frame="franka_tool", to_frame="world")
    # current_pose = fa.get_pose()
    # rot = quaternion.as_quat_array(np.array([0.012, -0.713, -0.700, -0.012]))
    # des_pose1 = RigidTransform(rotation = quaternion.as_rotation_matrix(rot),
    #                           translation= np.array([ 0.5 ,0,  0.507 ]), 
    #                           from_frame='franka_tool', to_frame='world')
    # des_pose2 = RigidTransform(rotation = quaternion.as_rotation_matrix(rot),
    #                           translation= np.array([ 0.5 ,0,  0.007 ]), 
    #                           from_frame='franka_tool', to_frame='world')
    # print(current_pose)
    # fa.goto_pose(des_pose1, use_impedance=False, duration = 10)
    # fa.goto_pose(des_pose2, use_impedance=False, duration = 10)
    
