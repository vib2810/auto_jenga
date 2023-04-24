#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
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


class Image_Proc():

  def __init__(self):
    self.bridge = CvBridge()
    color_topic = "/rgb/image_raw"
    depth_topic = "/depth_to_rgb/image_raw"

    #load params
    self.depth_threshold=15
    self.lower_hsv = (9, 20, 20)
    self.upper_hsv = (16, 100, 255)
    self.block_height= 15
    self.mask_pixels = 3250
    self.num_instances = 0
    
    #write code for message filters subscriber for depth and color images
    self.color_sub=message_filters.Subscriber(color_topic, Image)
    self.depth_sub=message_filters.Subscriber(depth_topic, Image)

    rospy.loginfo(rospy.get_caller_id() + "Subscribed to topics!!!")

    ts=message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],10,1)
    ts.registerCallback(self.inference) 

  def create_sample_dict(self,image,num_instances=None,best_rect=None,mask=None,best_inlier_ratio=0):

    img_cpy = image.copy() if mask is None else mask.copy()
    num_instances = 1 if num_instances is None else num_instances
    height,width = image.shape[:2]
    num_pixels = np.sum(img_cpy==255)
    print("num_pixels",num_pixels)
    print(num_instances)
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
      # convert to Nx2 array
      points = np.column_stack((x,y)).tolist()
      print("Points",len(points))
      print("Mean points",np.array(points).mean(axis=0))
      print(center)
      points = [point for point in points if np.sqrt((point[0]-center[1])**2 + (point[1]-center[0])**2) < 50]
      points = np.array(points)

      # print("Points",points.shape)

      params['min_inlier_ratio'] = 0.9
      params['num_iterations'] = 1000
      params['best_inlier_ratio'] = best_inlier_ratio
      params['best_rect'] = best_rect
      params['criterion'] = 'area'

      print("Inside second Racsac initialization")

    else:

      # sample x,y points from image where mask is 255
      x,y = np.where(image==255) 
      # convert to Nx2 array
      points = np.column_stack((x,y))

      theta_steps = 37
      theta_array = np.linspace(0,2*np.pi,theta_steps)
      params['min_inlier_ratio'] = 0.9
      params['num_iterations'] = 500
      params['best_inlier_ratio'] = best_inlier_ratio
      params['best_rect'] = None
      params['criterion'] = 'inliers'

    params['theta_array'] = theta_array
    params['points'] = points
    params['l_mean'] = l_mean
    params['w_mean'] = w_mean
    params['l_threshold'] = 0.1
    params['w_threshold'] = 0.1
    params['num_instances'] = num_instances
          
    return params

  def ransac_rect(self,color_img,image,num_instances=1,params:dict ={}):
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

    iteration=0
    found = False
    height,width = image.shape[:2]
    best_inlier_ratio=params['best_inlier_ratio']

    #EXTRACT PARAMS
    theta_array = params['theta_array']
    points = params['points']
    l_mean = params['l_mean']
    w_mean = params['w_mean']
    l_threshold = params['l_threshold']
    w_threshold = params['w_threshold']
    num_instances = params['num_instances']
    min_inlier_ratio = params['min_inlier_ratio']
    best_rect = params['best_rect'] if params['best_rect'] is not None else None
    criterion = params['criterion']

    while(iteration<params['num_iterations'] and not found):

      iteration+=1
      print("Iteration",iteration)  
      black_img = np.zeros((height,width,3), np.uint8)

      point_idx = np.random.choice(points.shape[0], 1, replace=False)
      theta_idx = np.random.choice(theta_array.shape[0], 1, replace=False)

      sampled_point = points[point_idx].squeeze(0)
      sampled_theta = theta_array[theta_idx]
      #randomly sample length and width of rectangle
      sampled_l = np.random.uniform((1-l_threshold)*l_mean, (1+l_threshold)*l_mean)
      sampled_w = np.random.uniform((1-w_threshold)*w_mean, (1+w_threshold)*w_mean)

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

      # self.plot(detection[:,:,0],title="Detection first channel")

      # self.plot(image,title="Depth Image")

      # self.plot(intersection*255,title="test first")

      # self.plot(detection,title="RANSAC detection Full firsr")

      #compute inliers ratio with the rectangle area
      inliers_ratio = num_inliers/(sampled_l*sampled_w)

      

      if(criterion=='inliers'):
        if(inliers_ratio>=best_inlier_ratio):
          best_inlier_ratio = inliers_ratio
          best_rect = [center,size,angle] #image coordinate x1,y1,x2,y2

        # if(best_inlier_ratio>=min_inlier_ratio):
        #   self.plot(detection,title="RANSAC detection")
        #   found=True

    print("Best inlier ratio",best_inlier_ratio)

    rect = cv2.boxPoints((best_rect[0], best_rect[1], best_rect[2]))
    rect = np.int0(rect)

    draw_box = cv2.drawContours(color_img, [rect], 0, (255, 255, 255), 2)
    self.plot(draw_box,title="RANSAC detection plot colored first")
    res=dict()
    res['best_rect'] = best_rect
    res['best_inlier_ratio'] = best_inlier_ratio
    res['mask'] = detection[:,:,0]

    return res


  def ransac_refine_rect(self,color_img,image,num_instances=1,initial_area=0,params:dict ={}):
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
      print("No params passed")
      params = self.create_sample_dict(image,num_instances)

    iteration=0
    found = False
    height,width = image.shape[:2]
    best_inlier_ratio=0

    #EXTRACT PARAMS
    theta_array = params['theta_array']
    points = params['points']
    # l_mean = params['l_mean']
    # w_mean = params['w_mean']
    l_threshold = params['l_threshold']
    w_threshold = params['w_threshold']
    num_instances = params['num_instances']
    min_inlier_ratio = params['min_inlier_ratio']
    best_rect = params['best_rect'] if params['best_rect'] is not None else None
    criterion = params['criterion']

    l_mean = best_rect[1][1]
    w_mean = best_rect[1][0]

    #print all params in best_rect
    print("printing all prams in best_rect-------------------")
    print("Center",best_rect[0])
    print("size",best_rect[1])
    print("angle",best_rect[2])


    best_area = initial_area  

    while(iteration<params['num_iterations'] and not found):

      iteration+=1
      print("Iteration",iteration)  
      black_img = np.zeros((height,width,3), np.uint8)

      # point_idx = np.random.choice(points.shape[0], 1, replace=False)
      sampled_pointx = np.random.uniform(low = best_rect[0][1]-10,high = best_rect[0][1]+10)
      sampled_pointy = np.random.uniform(low = best_rect[0][0]-10,high = best_rect[0][0]+10)
      sampled_point = np.array([sampled_pointx,sampled_pointy])
      theta_idx = np.random.choice(theta_array.shape[0], 1, replace=False)

      # sampled_point = points[point_idx].squeeze(0)
      sampled_theta = theta_array[theta_idx]
      #randomly sample length and width of rectangle
      sampled_l = np.random.uniform(low = (1-l_threshold)*l_mean, high = (1+l_threshold)*l_mean)
      sampled_w = np.random.uniform(low = (1-w_threshold)*w_mean, high = (1+w_threshold)*w_mean)
      # sampled_l = 100
      # sampled_w = 100
      print("Sampled l",sampled_l, "sampled W", sampled_w)
      #uniformly sample length and width of rectangle



      # Define the center and size of the rotated rectangle
      center = (sampled_point[1],sampled_point[0])
      size = (sampled_w, sampled_l)
      angle = sampled_theta * 180 / np.pi

      print("Angle=",angle)

      # Calculate the four corners of the rotated rectangle
      rect = cv2.boxPoints((center, size, angle))
      rect = np.int0(rect)

      detection = cv2.drawContours(black_img, [rect], 0, (255, 255, 255), -1)

      # draw_box = cv2.drawContours(color_img, [rect], 0, (255, 255, 255), 2)

      # self.plot(draw_box,title="RANSAC detection plot colored")
      # detection = cv2.circle(detection,(center[0],center[1]), 5, (0,0,255), -1)
      # self.plot(detection,title="RANSAC detection plot")

      # self.plot(detection,title="RANSAC detection Full second")

      # self.plot(detection[:,:,0],title="first channel")
      intersection  = (detection[:,:,0]/255)*(image/255)
    
      #compute inliers with the image
      num_inliers = np.sum(intersection)

      #compute inliers ratio with the rectangle area
      inliers_ratio = num_inliers/(sampled_l*sampled_w)

      #compute area of detection
      area = sampled_l*sampled_w
      print("Area",area, best_area)
      if(area>best_area and inliers_ratio>=0.98):
        print("Old area", best_area, "New area",area)
        best_area = area
        best_inlier_ratio = inliers_ratio
        best_rect = [center,size,angle]
        self.plot(intersection*255,title="test second")
        self.plot(image,title="Depth Image Second")
        self.plot(detection,title="RANSAC detection Full second")


    print("Best inlier ratio",best_inlier_ratio)

    if(best_area):
      print("Best_area:",best_area)

    rect = cv2.boxPoints((best_rect[0], best_rect[1], best_rect[2]))
    rect = np.int0(rect)

    draw_box = cv2.drawContours(color_img, [rect], 0, (255,0, 0), 2)
    self.plot(draw_box,title="RANSAC detection plot colored second")
    res=dict()
    res['best_rect'] = best_rect
    res['best_inlier_ratio'] = best_inlier_ratio
    res['mask'] = detection[:,:,0]

    return res

  def draw_harris_corners(self,color_img,depth_mask):

    gray = np.float32(depth_mask.copy())
    dst = cv2.cornerHarris(gray,3,5,0.04)
    dst = cv2.dilate(dst,None)
    image = color_img.copy()
    image[dst>0.01*dst.max()]=[0,0,255]
    self.plot(image,title="Harris Corners")
    return image

  def compute_k_means(self,depth_mask,color_img):

    self.num_instances = np.round((depth_mask.sum()/255)/self.mask_pixels)
    print("Num instances=",self.num_instances)

    indices = np.argwhere(depth_mask>0).reshape(-1,2)
    assert indices.shape[0] == depth_mask.sum()//255, f"indices shape={indices.shape} while depth mask is {depth_mask.sum()//255}"
    
    Z = np.float32(indices)

    #centering Z data and scaling
    mean = Z.mean(axis=0)
    std = Z.std(axis=0)
    Z[:,0] = Z[:,0] - mean[0]
    Z[:,1] = Z[:,1] - mean[1]
    Z[:,0] = Z[:,0]/std[0]
    Z[:,1] = Z[:,1]/std[1]

    #compute using open cv

    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 0.1)
    # K = int(self.num_instances)
    # ret,label,centers=cv2.kmeans(Z,K,None,criteria,1000,cv2.KMEANS_RANDOM_CENTERS)
    # centers = np.uint8(centers)
    # print(centers)

    #compute using scipy
    centers,label = kmeans2(Z, 4,iter=100, minit='random')

    centers = centers*std + mean

    #plot centers on depth_mask image
    for center in centers:
      image = cv2.circle(color_img, (int(center[1]),int(center[0])), 5, (255,0,0), -1)

    self.plot(image,type="gray",title="K-Means")

  def inference(self,color,depth):

    rospy.loginfo(rospy.get_caller_id() + "Images received!!!")

    try:
       color_img = self.bridge.imgmsg_to_cv2(color, "bgr8")
       depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")
       
    except CvBridgeError as e:
        print(e)

    #preprocess image
    color_img, depth_img,depth_mask = self.preprocess_image(color_img, depth_img)

    self.image=color_img

    #max and min value of depth image
    # print(depth_img.min(), depth_img.max())

    # self.plot(depth_img,type="colormap",title="Depth Image")
    # self.plot(depth_img,title="Depth Image")
    # self.plot(color_img,title="Color Image")
    self.plot(depth_mask,type="gray",title="Depth Mask")
    #print non zeros in depth_mask
    # print(np.count_nonzero(depth_mask))

    #compute HSV mask
    # hsv_mask = self.compute_hsv_mask(color_img,self.lower_hsv,self.upper_hsv)
    # self.plot(hsv_mask,title="HSV Mask")

    #draw contours
    # depth_mask = self.erode(depth_mask,9)

    #draw harris corners
    # self.draw_harris_corners(color_img, depth_mask)

    # print("count white pixels=",(depth_mask.sum()/255)//4)
    # contours = self.get_contours(depth_mask)

    #rectanlges using chatgpt
    num_instances = np.round((depth_mask.sum()/255)/self.mask_pixels)
    num_instances = 4
    print("Num instances=",num_instances)
    res = self.ransac_rect(color_img,depth_mask,num_instances)

    print("Finished coarse ransac")
    cv2.waitKey(0)

    params = self.create_sample_dict(depth_mask,num_instances,best_rect=res['best_rect'],mask=res['mask'],best_inlier_ratio=res['best_inlier_ratio'])
    res = self.ransac_refine_rect(color_img,depth_mask,initial_area = res['best_rect'][1][0]*res['best_rect'][1][1],params=params)

    # color_mask = self.draw_countours(color_img,contours,box=True)
    # self.plot(color_mask,title="Color Mask")

    #apply K means on depth mask
    # self.compute_k_means(depth_mask,color_img)

    rate = rospy.Rate(10) # 10hz
    rate.sleep()
    cv2.waitKey(0)

def main():
  rospy.init_node('image_converter', anonymous=True)
  try:
    image_logger = Image_Proc()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main()