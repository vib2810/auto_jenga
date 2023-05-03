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
        color_topic = rospy.get_param("instance_segmentor/color_topic")
        depth_topic = rospy.get_param("instance_segmentor/depth_topic")
        intrinsic_topic = rospy.get_param("instance_segmentor/intrinsic_topic")
        checkpoint_path = rospy.get_param("instance_segmentor/checkpoint_path")
        self.factor_depth = rospy.get_param("instance_segmentor/factor_depth")
        self.block_height = rospy.get_param("instance_segmentor/block_height")
        self.log = rospy.get_param("instance_segmentor/log")

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
        ts.registerCallback(self.get_images)

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
        
    def start(self):
        while not self.obtainedInitialImages and not self.obtainedIntrinsics:
            rospy.logdebug(f"[{rospy.get_name()}] " + " Waiting to start server")
            rospy.sleep(1)

        self._as.start()
        rospy.loginfo(f"[{rospy.get_name()}] " + " Grasp Pose Server Started")
    
    # global called  
    # called = False
    def inference(self, goal):
        if(self.obtainedInitialImages==False or self.obtainedIntrinsics==False):
            rospy.logerr("Initial images or intrinsics not obtained")
            # set status failed
            self._as.set_aborted()
            return
        # global called
        # if(called==True):
        #     return
        # called = True

        camera = CameraInfo_obj(
            self.depth_img.shape[1],
            self.depth_img.shape[0],
            self.intrinsics[0][0],
            self.intrinsics[1][1],
            self.intrinsics[0][2],
            self.intrinsics[1][2],
            self.factor_depth,
        )

        # center crop the image
        color_img_cropped = self.color_img[:, 280:1000, :]
        color_img_ = cv2.resize(color_img_cropped, (640, 640))

        #Get predictions from YOLO
        self.model.predict(color_img_,conf=0.5)
        results = self.model(color_img_)
        masks_ = results[0].masks.data  # raw masks tensor, (N, H, W) or masks.masks 
        masks_=masks_.to("cpu").numpy()

        #resize the detections onto the original image shape
        masks = []
        for idx, mask in enumerate(masks_):
            mask = mask.astype(np.uint8)
            mask = cv2.resize(mask, (720, 720))
            mask = np.pad(mask, ((0, 0), (280, 280)), "constant", constant_values=0) # zero padding at the ends
            masks.append(mask)
        
        masks = np.array(masks)

        # #compute pcd
        pcd = create_point_cloud_from_depth_image(self.depth_img, camera, organized=True)
        pcd = pcd.astype(np.float32)

        best_mask, best_mask_id = compute_best_mask(mask_arr=masks,pointcloud=pcd)
        print("Best Mask ID: ", best_mask_id)   
        if(best_mask_id==-1):
            rospy.logerr("No blocks detected")
            self._as.set_aborted()
            return
        compute_best_mask_2d(masks, pcd)

        #find cropped point cloud using best_mask
        idx = best_mask == 1
        best_mask= cv2.erode(best_mask.astype(np.uint8),np.ones((5,5),np.uint8),iterations = 2)
        pcd_cropped = pcd[best_mask == 1, :]

        #find centroid and orientation of block
        block_pose_base = compute_pose(pcd_cropped) #pose stamped

        self._result.pose = block_pose_base

        #convert quaternion to roll,pitch,yaw
        r = R.from_quat([block_pose_base.pose.orientation.x, block_pose_base.pose.orientation.y, block_pose_base.pose.orientation.z, block_pose_base.pose.orientation.w])
        roll, pitch, yaw = r.as_euler('zyx', degrees=True)
        angle = (roll,pitch,yaw)

        if self.log:
            # print("mask shape=",masks.shape)
            super_mask = self.color_img.copy().astype(np.int32)
            for mask in masks:
                super_mask[mask==1,:] += [0,0,50]
            super_mask[best_mask==1,:] += [0,50,0]
            super_mask = np.clip(super_mask,0,255).astype(np.uint8)
            plot(super_mask,type="normal",title="Super Mask")
            # plot_pcd(pcd_cropped,centroid = centroid,angle=angle)

            # plot_pcd(pcd)
        
        # Set action as being completed
        self._as.set_succeeded(self._result)

def main():
    try:
        segmenter = InstanceSegmenter()
        rospy.spin()
    except KeyboardInterrupt:
        # close all windows
        cv2.destroyAllWindows()
        print("Shutting down")


if __name__ == "__main__":
    main()
