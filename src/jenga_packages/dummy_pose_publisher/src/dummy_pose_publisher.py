#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws/src/git_packages/frankapy")
import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm
from block_pose_planner.srv import *

blocks = [PoseStamped() for i in range(3)]
blocks[0].header.frame_id = 'panda_link0'
blocks[0].pose.position.x = 0.415
blocks[0].pose.position.y = -0.2501
blocks[0].pose.position.z = 0.0079
blocks[0].pose.orientation.x = 0.9997
blocks[0].pose.orientation.y = -0.002
blocks[0].pose.orientation.z = -0.013
blocks[0].pose.orientation.w = 0.016

blocks[1].header.frame_id = 'panda_link0'
blocks[1].pose.position.x = 0.5178
blocks[1].pose.position.y = -0.2538
blocks[1].pose.position.z = 0.0079
blocks[1].pose.orientation.x = 0.9996
blocks[1].pose.orientation.y = 0.0069
blocks[1].pose.orientation.z = 0.0164
blocks[1].pose.orientation.w = 0.0168

blocks[2].header.frame_id = 'panda_link0'
blocks[2].pose.position.x = 0.6004
blocks[2].pose.position.y = -0.255
blocks[2].pose.position.z = 0.00746
blocks[2].pose.orientation.x = 0.9987
blocks[2].pose.orientation.y = -0.015
blocks[2].pose.orientation.z = 0.046
blocks[2].pose.orientation.w = 0.0089


def dummy_client(b_id = 0, l_id = 0):
    global blocks
    rospy.wait_for_service('go_to_block')
    if b_id == -1:
        des_pose = PoseStamped()
    else:
        des_pose = blocks[b_id]
    
    try:
        move_block = rospy.ServiceProxy('go_to_block', block_pose)
        resp = move_block(des_pose, b_id, l_id)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

if __name__ == '__main__':
    rospy.init_node('dummy_pose_publisher')
    block_count = 0
    layer_count = 0
    while(layer_count < 8):
        while(block_count <3):
            try:
                flag = dummy_client(block_count, layer_count)
                if(flag):
                    print("Block %d moved successfully" %block_count)
                    block_count += 1
            except rospy.ROSInterruptException:
                pass
        
        print("Layer %d completed" %layer_count)
        block_count = 0
        layer_count += 1

