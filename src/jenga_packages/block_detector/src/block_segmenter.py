#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters
import json
import argparse
import actionlib
import open3d as o3d
import scipy
import torch
from jenga_msgs.msg import GetBlocksAction, GetBlocksResult
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from select import select
from scipy.cluster.vq import kmeans2
import sys
sys.path.append("/home/ros_ws/src/jenga_packages/block_detector/src/")
from utils import *
from utils import CameraInfo_obj
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO
from PIL import Image
from geometry_msgs.msg import PoseStamped

"""
Implements a ransac based appraoch for detecting jenga blocks in the image. ROS service to relay the graspable pose of the block.

1) Preprocess the depth and color image
2) Run a coarse ransac to find a near optimal jenga block rectangle on masked image
3) Refine the coarse estimate with a 'fine ransac' that improves inlier ratio and maximizes area fit of the rectangle
4) Compute the graspable pose of the block and publish it to the client
"""


class InstanceSegmenter:
    def __init__(
        self,
    ):

        rospy.init_node("Instance Segmentor Node", anonymous=True)
        # Initialize action server
        self._as = actionlib.SimpleActionServer(
            "grasp_pose", GetBlocksAction, execute_cb=self.inference, auto_start=False
        )

        self._result = GetBlocksResult()

        self.bridge = CvBridge()
        self.block_height = 15
        color_topic = rospy.get_param("instance_segmentor/color_topic")
        depth_topic = "instance_segmentor/depth_topic"
        intrinsic_topic = "instance_segmentor/intrinsic_topic"
        checkpoint_path = "instance_segmentor/checkpoint_path"

        self.obtainedInitialImages=False
        self.obtainedIntrinsics=False

        # write code for message filters subscriber for depth and color images
        self.color_sub = message_filters.Subscriber(color_topic, Image)
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)
        self.cameraInfoSub = rospy.Subscriber(
            intrinsic_topic, CameraInfo, self.getCamInfo
        )
        rospy.loginfo(rospy.get_caller_id() + "Subscribed to topics!!!")

        self.model = YOLO(checkpoint_path)
        self.model.to("cuda")
        self.results=[]

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 10, 1
        )
        self.start()

        ts.registerCallback(self.inference)

    def get_images(self, color, depth):

        try:
            self.color_img = self.bridge.imgmsg_to_cv2(color, "bgr8")
            self.depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            self.obtainedInitialImages = True

        except CvBridgeError as e:
            self.obtainedInitialImages = False
            print(e)

    def getCamInfo(self, msg):
        "Get the camera intrinsics from the camera_info topic"

        self.intrinsics = np.array(msg.K).reshape(3, 3)
        self.obtainedIntrinsics = True
        rospy.logdebug(f"[{rospy.get_name()}] " + "Obtained camera intrinsics")
        self.cameraInfoSub.unregister()

    def publish_result(self,centroid,quat):

        #publish results
        self._result.pose.header.frame_id = "camera_color_optical_frame"
        self._result.pose.header.stamp = rospy.Time.now()
        self._result.pose.pose.position.x = centroid[0]
        self._result.pose.pose.position.y = centroid[1]
        self._result.pose.pose.position.z = centroid[2]
        self._result.pose.pose.orientation.x = quat[0]
        self._result.pose.pose.orientation.y = quat[1]
        self._result.pose.pose.orientation.z = quat[2]
        self._result.pose.pose.orientation.w = quat[3]
        
    def start(self):

        while not self.obtainedInitialImages and not self.obtainedIntrinsics:
            rospy.logdebug(f"[{rospy.get_name()}] " + " Waiting to start server")
            rospy.sleep(1)

        self._as.start()
        rospy.loginfo(f"[{rospy.get_name()}] " + " Grasp Pose Server Started")

    def inference(self, color, depth):

        rospy.loginfo(rospy.get_caller_id() + "Images received!!!")
        try:
            color_img = self.bridge.imgmsg_to_cv2(color, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")

        except CvBridgeError as e:
            print(e)

        #visualize color and depth image
        # cv2.imshow("color", color_img)
        # cv2.imshow("depth", depth_img)
        # cv2.waitKey(0)

        camera = CameraInfo_obj(
            depth_img.shape[1],
            depth_img.shape[0],
            self.intrinsics[0][0],
            self.intrinsics[1][1],
            self.intrinsics[0][2],
            self.intrinsics[1][2],
            self.factor_depth,
        )

        print("Depth image before", depth_img.shape)
        # preprocess image
        #resize color image to (640,640)
        color_img_ = cv2.resize(color_img, (640, 640))
        depth_img_ = cv2.resize(depth_img, (640, 640))

        #Get predictions from YOLO
        self.model.predict(color_img_,conf=0.5)
        results = self.model(color_img)
        results = results.to('cpu')
        masks = results[0].masks.data  # raw masks tensor, (N, H, W) or masks.masks 
        masks=masks.to("cpu").numpy()

        for idx,mask in enumerate(masks):
            mask = mask.astype(np.uint8)
            mask = cv2.resize(mask, (depth_img.shape[0], depth_img.shape[1]))
            masks[idx] = mask
        
        #compute pcd
        pcd = create_point_cloud_from_depth_image(depth_img, camera, organized=True)
        pcd = pcd.astype(np.float32)

        best_mask = compute_best_mask(mask_arr=masks,pointcloud=pcd)

        #find cropped point cloud using best_mask
        pcd_cropped = pcd[best_mask == 1, :]

        #find centroid and orientation of block
        centroid,quat = compute_pose(pcd_cropped)

        # adjust for block height
        centroid[2] = centroid[2] + self.block_height / 1000

        #publish result
        self.publish_result(centroid,quat)

        #convert quaternion to roll,pitch,yaw
        r = R.from_quat(quat)
        roll, pitch, yaw = r.as_euler('zyx', degrees=True)
        angle = (roll,pitch,yaw)

        if self.log:
            plot_pcd(pcd_cropped,centroid = centroid,angle=angle)
            # plot_pcd(pcd)
        
        #Set action as being completed
        self._as.set_succeeded(self._result)

def main():

    try:
        segmenter = InstanceSegmenter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
