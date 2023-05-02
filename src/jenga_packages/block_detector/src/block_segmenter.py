#!/usr/bin/env python

import rospy
import rospkg
import yaml
import tf2_ros
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import Pose
import torch

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_node')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(torch.cuda.is_available())
        rate.sleep()

    