#!/usr/bin/env python

import rospy
# import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from move_it_block_pose_planner.srv import *
from frankapy import FrankaArm
import quaternion
from autolab_core import RigidTransform
from manipulation.src import demo_moveit as moveit
# 75mm length, 15mm height, 25 mm width

def pickup_block(req_pose):
    global fa
    global moveit_handler

    # Add block as obstacle to pick up 
    # x along length, y along width ,z along height - use half the distance accounts for symmetry
    moveit_handler.add_box(name="pickup_block", pose=req_pose, size=[0.075/2.0, 0.025/2.0, 0.015/2.0])
    _, plan = moveit_handler.get_plan_given_pose(req_pose)

    # Blocking function
    moveit_handler.execute_plan(plan)   
    fa.close_gripper()
    # moveit_handler.reset_joints()

    return True

def drop_block(block_id, layer_id):
    global fa
    global moveit_handler

    # Hard coded because staging area - need to update when testing with actual limits
    middle_block_pose_even ={'orientation': np.array([0.0, 1.0, 0.0, 0.0]), 
                        'translation': np.array([0.5, 0, 0])}
    middle_block_pose_odd = {'orientation': np.array([0.7071068, 0.0, 0.0, 0.7071068]), 
                        'translation': np.array([0.5, 0, 0])}
    if(layer_id%2 == 0):
        drop_pose_quat = quaternion.as_quat_array(middle_block_pose_even['orientation'])
        drop_pose_transl = middle_block_pose_even['translation'] + (1 - block_id%3)*np.array([0.025, 0, 0])
    else:
        drop_pose_quat = quaternion.as_quat_array(middle_block_pose_odd['orientation'])
        drop_pose_transl = middle_block_pose_odd['translation'] + (1 - block_id%3)*np.array([0, 0.025, 0])
    
    # pose_to_drop = RigidTransform(rotation=quaternion.as_rotation_matrix(drop_pose_quat), translation = drop_pose_transl, from_frame="franka_tool", to_frame="world")
    
    # fa.goto_pose(pose_to_drop, use_impedance=False, duration=15)

    drop_pose_transl = drop_pose_transl + (2*layer_id+1)*np.array([0, 0, 0.007])
    # pose_to_drop_local = RigidTrans form(rotation=quaternion.as_rotation_matrix(drop_pose_quat), translation = drop_pose_local_transl, from_frame="franka_tool", to_frame="world")
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "panda_link0" 
    goal_pose.pose.position.x = drop_pose_transl[0]
    goal_pose.pose.position.y = drop_pose_transl[1]
    goal_pose.pose.position.z = drop_pose_transl[2]

    goal_pose.pose.orientation.w = drop_pose_quat[0]
    goal_pose.pose.orientation.x = drop_pose_quat[1]
    goal_pose.pose.orientation.y = drop_pose_quat[2]
    goal_pose.pose.orientation.z = drop_pose_quat[3]

    _, plan = moveit_handler.get_plan_given_pose(goal_pose)
    moveit_handler.execute_plan(plan)
    moveit_handler.add_box(name="block_"+str(block_id)+"_"+str(layer_id),pose=goal_pose, size=[0.075/2.0, 0.025/2.0, 0.015/2.0])
    fa.open_gripper()

    return True
    


def recv_pose_callback(req):
    global fa
    global moveit_handler

    print("Received pose information")
    req_pose = req.des_pose
    pickup_done = pickup_block(req_pose)
    if(pickup_done):
        moveit_handler.remove_box("pickup_block")
        drop_block(req.block_id, req.layer_id)

    return block_poseResponse(True)

def GoToBlockServer():
    s = rospy.Service('go_to_block', block_pose, recv_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    global moveit_handler

    rospy.init_node('block_pose_planner')
    moveit_handler = moveit.moveit_planner()
    GoToBlockServer()
    
    # des_pose = RigidTransform(rotation=np.eye(3), translation=np.array([0.4, 0, 0.165]), from_frame="franka_tool", to_frame="world")
    # current_pose = fa.get_pose()
    # des_pose = RigidTransform(rotation = np.array([[ 0.99735518,  0.00172061 , 0.07252959], [-0.00637742, -0.99375986,  0.11127286], [ 0.07226846, -0.11144111, -0.99113965]]),
    #                         #   translation= np.array([ 0.45871361 ,-0.03142103,  0.0412673 ]), 
    #                           from_frame='franka_tool', to_frame='world')
    # print(current_pose)
    # fa.goto_pose(des_pose, use_impedance=False, duration = 10)
    
