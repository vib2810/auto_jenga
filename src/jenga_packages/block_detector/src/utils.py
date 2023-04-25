import rospy
import cv2
import numpy as np
import message_filters
import json
import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from rospy.numpy_msg import numpy_msg
from select import select
from scipy.cluster.vq import kmeans2


"""Utility functions for implementing ransac and block detection using Open CV"""

class CameraInfo_obj:
    """ Camera intrisics for point cloud creation. """
    def __init__(self, width, height, fx, fy, cx, cy, scale):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.scale = scale

def plot(image,type="normal",title="Image"):
    if type=="colormap":
        image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03),  cv2.COLORMAP_HOT)

    elif(title=="HSV Mask"):
        cv2.imshow(title, image)
        cv2.waitKey(3)
        # cv2.createTrackbar('slider', title, 15, 70, hsv_tuning)

    cv2.imshow(title, image)
    cv2.waitKey(3)

def opening_morphology(image,kernel_size=5):
    kernel = np.ones((kernel_size,kernel_size),np.uint8)
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return opening

def erode(mask,kernel):
    kernel = np.ones((kernel,kernel),np.uint8)
    erosion = cv2.erode(mask,kernel)
    return erosion
    
def preprocess_image(color_img:np.ndarray=None,depth_img:np.ndarray=None,block_height:int=15,depth_threshold:int=15, bounds:tuple=(200,720,200,640)):

    """
    Function: Preprocesses the color and depth image by cropping and depth thresholding.
    """
    x_min,x_max,y_min,y_max = bounds
    
    color_img_cpy = color_img.copy()
    depth_img_cpy = depth_img.copy()
    color_img_cpy = color_img_cpy[x_min:x_max,y_min:y_max] #crop the image
    depth_img_cpy = depth_img_cpy[x_min:x_max,y_min:y_max]

    print (depth_img_cpy.min(),depth_img_cpy.max())
    depth_img_cpy[depth_img_cpy<depth_threshold]=depth_img_cpy.max() #filter erroenous depth values

    depth_mask = np.zeros_like(depth_img_cpy)
    min_depth = depth_img_cpy.min()
    print("min_depth=",min_depth)
    depth_mask[np.logical_and(depth_img_cpy>min_depth,depth_img_cpy<min_depth+block_height)]=255 #create a mask for the top layer block
    depth_mask=depth_mask.astype(np.uint8)
    print("depth_mask sum=",depth_mask.sum()//255)
    depth_mask = opening_morphology(depth_mask)

    return color_img_cpy, depth_img_cpy,depth_mask

def compute_hsv_mask(color_img,lower,upper):
    hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = mask = cv2.inRange(hsv_image,lower, upper)
    return mask

def get_contours(mask,mask_pixels=3250):

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # Define a minimum contour area threshold (change as necessary)
    min_contour_area = mask_pixels/1.5
    # Filter the contours based on their size
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_contour_area]
    return filtered_contours

def draw_countours(image,contours,box:bool=False):
    mask = []

    if box:
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            bbox = cv2.boxPoints(rect)
            bbox = np.int0(bbox)
            mask= cv2.drawContours(image,[bbox],0,(0,0,255),2)
    else:
        mask = cv2.drawContours(image, contours, -1, (0,255,0), 3)
    return mask

def draw_harris_corners(color_img,depth_mask):

    gray = np.float32(depth_mask.copy())
    dst = cv2.cornerHarris(gray,3,5,0.04)
    dst = cv2.dilate(dst,None)
    image = color_img.copy()
    image[dst>0.01*dst.max()]=[0,0,255]
    plot(image,title="Harris Corners")

    return image

def compute_k_means(depth_mask,color_img,num_clusters=4):

    # self.num_instances = np.round((depth_mask.sum()/255)/self.mask_pixels)
    # print("Num instances=",self.num_instances)

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

    #compute using scipy
    centers,label = kmeans2(Z, num_clusters,iter=100, minit='random')

    centers = centers*std + mean

    #plot centers on depth_mask image
    for center in centers:
        image = cv2.circle(color_img, (int(center[1]),int(center[0])), 5, (255,0,0), -1)

    plot(image,type="gray",title="K-Means")

def create_point_cloud_from_depth_image(depth, camera, organized=True):
    """ Generate point cloud using depth image only.

        Input:
            depth: [numpy.ndarray, (H,W), numpy.float32]
                depth image
            camera: [CameraInfo]
                camera intrinsics
            organized: bool
                whether to keep the cloud in image shape (H,W,3)

        Output:
            cloud: [numpy.ndarray, (H,W,3)/(H*W,3), numpy.float32]
                generated cloud, (H,W,3) for organized=True, (H*W,3) for organized=False
    """
    assert(depth.shape[1] == camera.width and depth.shape[0] == camera.height)
    # xmap = np.arange(-camera.width//2, camera.width//2)

    #camera width =1280
    #camera height = 720
    xmap = np.arange(0, camera.width)
    ymap = np.arange(camera.height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depth / camera.scale
    points_x = (xmap - camera.cx) * points_z / camera.fx
    points_y = (ymap - camera.cy) * points_z / camera.fy
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    if not organized:
        cloud = cloud.reshape([-1, 3])
    return cloud


def create_geometry_at_points(points):
    geometries = o3d.geometry.TriangleMesh()
    for point in points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05) #create a small sphere to represent point
        sphere.translate(point) #translate this sphere to point
        geometries += sphere
    geometries.paint_uniform_color([1.0, 0.0, 0.0])
    return geometries


def plot_pcd(points):

    points = points.reshape((-1,3))
    plane_cloud = o3d.geometry.PointCloud()                 # creating point cloud of plane
    plane_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([plane_cloud])