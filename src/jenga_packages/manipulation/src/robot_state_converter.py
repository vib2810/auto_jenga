#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

sys.path.append("/home/ros_ws/src/jenga_packages/frankapy")
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import min_jerk

gripper_width = 0
def gripper_callback(msg):
    global gripper_width
    gripper_width = msg.position[0]

if __name__ == "__main__":
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    fa = FrankaArm(init_node = False)
    # setup publisher
    pub = rospy.Publisher("/real_robot_joints", sensor_msgs.msg.JointState, queue_size=10)
    gripper_sub = rospy.Subscriber("/franka_gripper_1/joint_states", sensor_msgs.msg.JointState, gripper_callback)
    rate = rospy.Rate(10)
    msg = sensor_msgs.msg.JointState()
    msg.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"]
    while not rospy.is_shutdown():
        state = fa.get_joints().tolist()
        msg.header.stamp = rospy.Time.now()
        state.append(gripper_width); state.append(gripper_width)
        msg.position = state
        pub.publish(msg)
        rate.sleep()