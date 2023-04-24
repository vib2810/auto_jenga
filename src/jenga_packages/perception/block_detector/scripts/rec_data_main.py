#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters
import json
import argparse
import actionlib
import open3d as o3d
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from select import select
from scipy.cluster.vq import kmeans2
from utils import *



"""
Implements a ransac based appraoch for detecting jenga blocks in the image. ROS service to relay the graspable pose of the block.

1) Preprocess the depth and color image
2) Run a coarse ransac to find a near optimal jenga block rectangle on masked image
3) Refine the coarse estimate with a 'fine ransac' that improves inlier ratio and maximizes area fit of the rectangle
4) Compute the graspable pose of the block and publish it to the client
"""


class RANSAC():

  def __init__(self,):

    rospy.init_node('Ransac Node', anonymous=True)
    #Initialize action server
    # self._as = actionlib.SimpleActionServer('grasp_pose', GraspPoseAction, execute_cb=self.inference, auto_start = False)

    self.bridge = CvBridge()
    color_topic = "/rgb/image_raw"
    depth_topic = "/depth_to_rgb/image_raw"
    intrinsic_topic = "/depth_to_rgb/camera_info"

    #load params
    self.factor_depth = rospy.get_param('/ransac/factor_depth')
    self.log = rospy.get_param('/ransac/log')
    self.depth_threshold = rospy.get_param('/ransac/depth_threshold')
    self.block_height = rospy.get_param('/ransac/block_height')
    x_min = rospy.get_param('/ransac/x_min')
    x_max = rospy.get_param('/ransac/x_max')
    y_min = rospy.get_param('/ransac/y_min')
    y_max = rospy.get_param('/ransac/y_max')

    #set params
    # self.lower_hsv = (9, 20, 20)
    # self.upper_hsv = (16, 100, 255)
    self.mask_pixels = 3250
    self.num_instances = 0
    self.bounds = (x_min, x_max, y_min, y_max)
    
    #write code for message filters subscriber for depth and color images
    self.color_sub=message_filters.Subscriber(color_topic, Image)
    self.depth_sub=message_filters.Subscriber(depth_topic, Image)
    self.cameraInfoSub = rospy.Subscriber(intrinsic_topic, CameraInfo, self.getCamInfo)
    rospy.loginfo(rospy.get_caller_id() + "Subscribed to topics!!!")

    ts=message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],10,1)
    ts.registerCallback(self.inference) 

  def get_images(self,color,depth):

    try:
      self.color_img = self.bridge.imgmsg_to_cv2(color, "bgr8")
      self.depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")
      self.obtainedInitialImages = True
       
    except CvBridgeError as e:
      self.obtainedInitialImages = False
      print(e)
  
  def getCamInfo(self,msg):
    "Get the camera intrinsics from the camera_info topic"

    self.intrinsics = np.array(msg.K).reshape(3,3)
    self.obtainedIntrinsics = True
    rospy.logdebug(f"[{rospy.get_name()}] " + "Obtained camera intrinsics")
    self.cameraInfoSub.unregister()

  def create_sample_dict(self,image,num_instances=None,best_rect=None,mask=None,best_inlier_ratio=0):

    img_cpy = image.copy() if mask is None else mask.copy()
    num_instances = 1 if num_instances is None else num_instances
    height,width = image.shape[:2]
    num_pixels = np.sum(img_cpy==255)
    pixels_per_instance = num_pixels/num_instances
    l_mean = int(3*np.sqrt(pixels_per_instance/3))
    w_mean = l_mean//3 

    params=dict()

    if(best_rect is not None and mask is not None):

      center,size,angle = best_rect[0], best_rect[1], best_rect[2]
      theta_steps = 100
      theta_array = np.linspace(angle*np.pi/180-np.pi/18,angle*np.pi/180+np.pi/18,theta_steps)

      # sample x,y points from image where mask is 255
      x,y = np.where(mask==255) 
      points = np.column_stack((x,y)).tolist()
      points = [point for point in points if np.sqrt((point[0]-center[1])**2 + (point[1]-center[0])**2) < 50]
      points = np.array(points)

      #store in params dict
      params['min_inlier_ratio'] = 0.98
      params['num_iterations'] = 1000
      params['best_inlier_ratio'] = best_inlier_ratio
      params['best_rect'] = best_rect
      params['criterion'] = 'area'
      params['l_mean'] = best_rect[1][1]
      params['w_mean'] = best_rect[1][0]
      params['theta_array'] = theta_array
      params['points'] = points
      params['l_threshold'] = 0.1
      params['w_threshold'] = 0.1
      params['num_instances'] = num_instances

    else:

      # sample x,y points from image where mask is 255
      x,y = np.where(image==255) 
      points = np.column_stack((x,y))

      theta_steps = 37
      theta_array = np.linspace(0,2*np.pi,theta_steps)

      #store in params dict
      params['min_inlier_ratio'] = 0.9
      params['num_iterations'] = 500
      params['best_inlier_ratio'] = best_inlier_ratio
      params['best_rect'] = None
      params['criterion'] = 'inliers'
      params['l_mean'] = l_mean
      params['w_mean'] = w_mean
      params['theta_array'] = theta_array
      params['points'] = points
      params['l_threshold'] = 0.1
      params['w_threshold'] = 0.1
      params['num_instances'] = num_instances
          
    return params

  def sample_params(self,params:dict={},mode:str='coarse'):

    if(mode=='coarse'):
      point_idx = np.random.choice(params['points'].shape[0], 1, replace=False)
      theta_idx = np.random.choice(params['theta_array'].shape[0], 1, replace=False)
      theta = params['theta_array'][theta_idx]
      point = params['points'][point_idx].squeeze(0)
      l = np.random.uniform((1-params['l_threshold'])*params['l_mean'], (1+params['l_threshold'])*params['l_mean'])
      w = np.random.uniform((1-params['w_threshold'])*params['w_mean'], (1+params['w_threshold'])*params['w_mean'])
      return (point,theta,l,w)

    elif(mode=='fine'):

      best_rect = params['best_rect']
      sampled_pointx = np.random.uniform(low = best_rect[0][1]-10,high = best_rect[0][1]+10)
      sampled_pointy = np.random.uniform(low = best_rect[0][0]-10,high = best_rect[0][0]+10)
      point = np.array([sampled_pointx,sampled_pointy])

      theta_idx = np.random.choice(params['theta_array'].shape[0], 1, replace=False)
      theta = params['theta_array'][theta_idx]
    
      l = np.random.uniform((1-params['l_threshold'])*params['l_mean'], (1+params['l_threshold'])*params['l_mean'])
      w = np.random.uniform((1-params['w_threshold'])*params['w_mean'], (1+params['w_threshold'])*params['w_mean'])
      return (point,theta,l,w)    

  def ransac_rect(self,color_img,image,num_instances=1,params:dict ={},mode: str = 'coarse',log: bool = False):
    """
    Uses RANSAC to fit a rectangle to an image.

    :param image: The input image.
    :param num_iterations: The maximum number of RANSAC iterations.
    :param min_num_inliers: The minimum number of inliers required to fit a rectangle.
    :param inlier_threshold: The maximum distance between a point and the rectangle for it to be considered an inlier.
    :return: The best rectangle found using RANSAC.
    """

    #if params is empty
    if not params:
      print("No params passed!!!")
      params = self.create_sample_dict(image,num_instances)

    #EXTRACT PARAMS
    theta_array = params['theta_array']
    points = params['points']
    l_mean = params['l_mean']
    w_mean = params['w_mean']
    l_threshold = params['l_threshold']
    w_threshold = params['w_threshold']
    num_instances = params['num_instances']
    min_inlier_ratio = params['min_inlier_ratio']
    best_inlier_ratio=params['best_inlier_ratio']
    best_rect = params['best_rect'] if params['best_rect'] is not None else None
    best_area = best_rect[1][0] * best_rect[1][1] if params['best_rect'] is not None else None

    iteration=0
    found = False
    height,width = image.shape[:2]

    while(iteration<params['num_iterations'] and not found):

      iteration+=1
      print(f"Mode: {mode},Iteration:",iteration)  
      black_img = np.zeros((height,width,3), np.uint8)

      sampled_point,sampled_theta,sampled_l,sampled_w = self.sample_params(params=params,mode=mode)

      # Define the center and size of the rotated rectangle
      center = (sampled_point[1],sampled_point[0])
      size = (sampled_w, sampled_l)
      angle = sampled_theta * 180 / np.pi

      # Calculate the four corners of the rotated rectangle
      rect = cv2.boxPoints((center, size, angle))
      rect = np.int0(rect)

      detection = cv2.drawContours(black_img, [rect], 0, (255, 255, 255), -1)
      # detection = cv2.circle(detection,(center[0],center[1]), 5, (0,0,255), -1)
    
      #compute inliers with the image
      intersection  = (detection[:,:,0]/255)*(image/255)
      num_inliers = np.sum(intersection)

      if(log):
        log_image = np.hstack((image,detection[:,:,0],intersection*255))
        plot(log_image,title=f"{mode} Ransac: Depth image, detection, intersection")

      #compute inliers ratio with the rectangle area
      inliers_ratio = num_inliers/(sampled_l*sampled_w)

      if(mode=='coarse'):
        if(inliers_ratio>=best_inlier_ratio):
          best_inlier_ratio = inliers_ratio
          best_rect = [center,size,angle] #image coordinate x1,y1,x2,y2

          # if(best_inlier_ratio>=min_inlier_ratio):
          #   self.plot(detection,title="RANSAC detection")
          #   found=True

      elif(mode=='fine'):
        area = sampled_l*sampled_w
        if(area>best_area and inliers_ratio>=params['min_inlier_ratio']):
          best_area = area
          best_inlier_ratio = inliers_ratio
          best_rect = [center,size,angle]

    rospy.loginfo(f"Best inlier ratio for mode {mode} ={best_inlier_ratio}")

    #draw best rectangle on the color image
    rect = cv2.boxPoints((best_rect[0], best_rect[1], best_rect[2]))
    rect = np.int0(rect)
    
    if(log):
      if(mode=='coarse'):
        draw_box = cv2.drawContours(color_img, [rect], 0, (255, 255, 255), 2)
      elif(mode=='fine'):
        draw_box = cv2.drawContours(color_img, [rect], 0, (255, 0, 0), 2)

      plot(draw_box,title=f"{mode} RANSAC detection colored")

    #store results
    res=dict()
    res['best_rect'] = best_rect
    res['best_inlier_ratio'] = best_inlier_ratio
    # res['mask'] = detection[:,:,0]
    black_img = np.zeros((height,width,3), np.uint8)
    mask = cv2.drawContours(black_img, [rect], 0, (255, 255, 255), -1)[:,:,0]
    res['mask'] = mask.copy()

    mask[mask==255] = 1
    res['depth_mask'] = mask*image

    return res

  def inference(self,color,depth):

    rospy.loginfo(rospy.get_caller_id() + "Images received!!!")
    try:
       color_img = self.bridge.imgmsg_to_cv2(color, "bgr8")
       depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")
       
    except CvBridgeError as e:
        print(e)

    camera = CameraInfo_obj(depth_img.shape[1], depth_img.shape[0], self.intrinsics[0][0], self.intrinsics[1][1], self.intrinsics[0][2], self.intrinsics[1][2], self.factor_depth)

    print("Depth image before",depth_img.shape)
    #preprocess image
    color_img_, depth_img_,depth_mask = preprocess_image(color_img, depth_img,block_height=15,depth_threshold=15,bounds=self.bounds)
    print("Depth img here",depth_img.shape)
    self.image=color_img_
 
    #rectanlges using chatgpt
    num_instances = np.round((depth_mask.sum()/255)/self.mask_pixels)
    num_instances = 4
    print("Num instances=",num_instances)
    res = self.ransac_rect(color_img_,depth_mask,num_instances,mode='coarse',log=self.log)

    print("Finished coarse ransac")
    cv2.waitKey(10)

    print("Results after coarse ransac",res['best_rect'])
    params = self.create_sample_dict(depth_mask,num_instances,best_rect=res['best_rect'],mask=res['mask'],best_inlier_ratio=res['best_inlier_ratio'])
    res = self.ransac_rect(color_img_,depth_mask,num_instances,params,mode='fine',log=self.log)

    cropped_depth_mask = res['depth_mask']
    yaw = (res['best_rect'][2] % 180)*np.pi/180

    #some data manip to account for size
    idx= list(np.where(cropped_depth_mask>0))
    depth_mask_ = np.zeros_like(depth_img)
    depth_mask_[idx[0]+200,idx[1]+200] = 1

    pcd = create_point_cloud_from_depth_image(depth_img, camera, organized=True)
    pcd = pcd.astype(np.float32)
    pcd_cropped = pcd[depth_mask_==1,:]

    centroid = np.median(pcd_cropped,axis=0)
    #adjust for block height
    centroid[2] = centroid[2] + self.block_height/1000

    print("Centroid",centroid)
    print("Yaw",yaw)

    if(self.log):
      plot_pcd(pcd_cropped)
      plot_pcd(pcd)

    #plot pcd

    # plot(res['mask'],title="Final mask")
    # color_mask = self.draw_countours(color_img_,contours,box=True)
    # self.plot(color_mask,title="Color Mask")

    #apply K means on depth mask
    # self.compute_k_means(depth_mask,color_img_)

    #find point cloud of cropped bbox here --use graspnet function and visualize it
    # pcd = create_point_cloud_from_depth_image(depth_img_mask, camera, organized=True)

    #find plane on the cropped depth image here
    #determine the normal and get grasp pose in camera frame here
    #publish the grasp pose here through ros actions
    #return server feedback

    rate = rospy.Rate(10) # 10hz
    rate.sleep()
    cv2.waitKey(0)


def start(self):

        while not self.obtainedInitialImages and not self.obtainedIntrinsics:
            rospy.logdebug(f"[{rospy.get_name()}] " + " Waiting to start server")
            rospy.sleep(1)

        self._as.start()
        rospy.loginfo(f"[{rospy.get_name()}] " + " Grasp Pose Server Started")


def main():
  
  try:
    image_logger = RANSAC()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main()