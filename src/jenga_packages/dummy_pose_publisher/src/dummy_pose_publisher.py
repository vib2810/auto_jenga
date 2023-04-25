#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws/src/git_packages/frankapy")
import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm
from block_pose_planner.srv import *

blocks = [PoseStamped() for i in range(3)]
blocks[0].header.frame_id = 'panda_link0'
blocks[0].pose.position.x = 0.3476
blocks[0].pose.position.y = -0.2658
blocks[0].pose.position.z = 0.007
blocks[0].pose.orientation.x = 0.9999
blocks[0].pose.orientation.y = -0.002
blocks[0].pose.orientation.z = -0.008
blocks[0].pose.orientation.w = 0.0039

blocks[1].header.frame_id = 'panda_link0'
blocks[1].pose.position.x = 0.5937
blocks[1].pose.position.y = -0.2634
blocks[1].pose.position.z = 0.007
blocks[1].pose.orientation.x = 0.885
blocks[1].pose.orientation.y = 0.4614
blocks[1].pose.orientation.z = 0.047
blocks[1].pose.orientation.w = -0.013

blocks[2].header.frame_id = 'panda_link0'
blocks[2].pose.position.x = 0.3476
blocks[2].pose.position.y = -0.2658
blocks[2].pose.position.z = 0.007
blocks[2].pose.orientation.x = 0.9999
blocks[2].pose.orientation.y = -0.002
blocks[2].pose.orientation.z = -0.008
blocks[2].pose.orientation.w = 0.0039


def dummy_client(b_id = 0, l_id = 0):
    global blocks
    rospy.wait_for_service('go_to_block')
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
    while(layer_count < 6):
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

