#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm
from block_pose_planner.srv import *

blocks = [PoseStamped() for i in range(3)]
blocks[0].header.frame_id = 'world'
blocks[0].pose.position.x = 0.507
blocks[0].pose.position.y = -0.318
blocks[0].pose.position.z = 0.006
blocks[0].pose.orientation.x = 0.944
blocks[0].pose.orientation.y = -0.320
blocks[0].pose.orientation.z = 0.065
blocks[0].pose.orientation.w = 0.003

blocks[1].header.frame_id = 'world'
blocks[1].pose.position.x = 0.370
blocks[1].pose.position.y = -0.238
blocks[1].pose.position.z = 0.006
blocks[1].pose.orientation.x = 0.999
blocks[1].pose.orientation.y = 0.019
blocks[1].pose.orientation.z = -0.013
blocks[1].pose.orientation.w = -0.003

blocks[2].header.frame_id = 'world'
blocks[2].pose.position.x = 0.603
blocks[2].pose.position.y = -0.211
blocks[2].pose.position.z = 0.006
blocks[2].pose.orientation.x = 0.728
blocks[2].pose.orientation.y = -0.682
blocks[2].pose.orientation.z = 0.051
blocks[2].pose.orientation.w = 0.023


def dummy_client(b_id = 0):
    global blocks
    rospy.wait_for_service('go_to_block')
    des_pose = blocks[b_id]
    try:
        move_block = rospy.ServiceProxy('go_to_block', block_pose)
        resp = move_block(des_pose, b_id)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

if __name__ == '__main__':
    rospy.init_node('dummy_pose_publisher')
    block_count = 0
    while(block_count <3):
        try:
            flag = dummy_client(block_count)
            if(flag):
                print("Block %d moved successfully" %block_count)
                block_count += 1
        except rospy.ROSInterruptException:
            pass

