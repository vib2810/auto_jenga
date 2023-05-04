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
from scipy.spatial.transform import Rotation as R

"""Utility functions for implementing ransac and block detection using Open CV"""

import scipy.spatial.transform as spt
import geometry_msgs.msg
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
import copy

# ---------------------Transformations---------------------
def pose_to_transformation_matrix(pose):
    """
    Converts geometry_msgs/Pose to a 4x4 transformation matrix
    """
    T = np.eye(4)
    T[0,3] = pose.position.x
    T[1,3] = pose.position.y
    T[2,3] = pose.position.z
    r = spt.Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T[0:3, 0:3] = r.as_matrix()
    return T

def transformation_matrix_to_pose(trans_mat):   
    """
    Converts a 4x4 transformation matrix to geometry_msgs/Pose
    """
    out_pose = geometry_msgs.msg.Pose()
    out_pose.position.x = trans_mat[0,3]
    out_pose.position.y = trans_mat[1,3]
    out_pose.position.z = trans_mat[2,3]

    #convert rotation matrix to quaternion
    r = spt.Rotation.from_matrix(trans_mat[0:3, 0:3])
    quat = r.as_quat() 
    out_pose.orientation.x = quat[0]
    out_pose.orientation.y = quat[1]
    out_pose.orientation.z = quat[2]
    out_pose.orientation.w = quat[3] 
    return out_pose

def transform_pcl_to_base(pointcloud: np.ndarray):
    """
    Transforms Nx3 pointcloud from camera_color_optical_frame to panda_link0
    """
    try:
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # rgb_to_cambase = tf_buffer.lookup_transform('camera_base', 'rgb_camera_link', rospy.Time(0), rospy.Duration(1.0))
        base_to_opt = tf_buffer.lookup_transform('panda_link0', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(10.0))
        base_to_opt_pose = Pose()
        base_to_opt_pose.position.x = base_to_opt.transform.translation.x
        base_to_opt_pose.position.y = base_to_opt.transform.translation.y
        base_to_opt_pose.position.z = base_to_opt.transform.translation.z
        base_to_opt_pose.orientation.x = base_to_opt.transform.rotation.x
        base_to_opt_pose.orientation.y = base_to_opt.transform.rotation.y
        base_to_opt_pose.orientation.z = base_to_opt.transform.rotation.z
        base_to_opt_pose.orientation.w = base_to_opt.transform.rotation.w
        
        base_to_opt_mat = pose_to_transformation_matrix(base_to_opt_pose)
        pointcloud = np.hstack((pointcloud,np.ones((pointcloud.shape[0],1)))) # shape (N,4)
        pointcloud = np.dot(base_to_opt_mat,pointcloud.T).T
        pointcloud = pointcloud[:,:3]
        return pointcloud
    except:
        print("tf lookup failed in block pose conversion")
        return None


def transform_cam_to_base(opt_to_block_pose_stamped: PoseStamped):
    """
    Transforms PoseStamped from camera_color_optical_frame to panda_link0
    """
    try:
        block_to_opt_pose = copy.deepcopy(opt_to_block_pose_stamped.pose)
        # get ros transform between camera_base and rgb_camera_link using ros tf api
        # create tf subscriber
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # rgb_to_cambase = tf_buffer.lookup_transform('camera_base', 'rgb_camera_link', rospy.Time(0), rospy.Duration(1.0))
        base_to_opt = tf_buffer.lookup_transform('panda_link0', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(10.0))
        base_to_opt_pose = Pose()
        base_to_opt_pose.position.x = base_to_opt.transform.translation.x
        base_to_opt_pose.position.y = base_to_opt.transform.translation.y
        base_to_opt_pose.position.z = base_to_opt.transform.translation.z
        base_to_opt_pose.orientation.x = base_to_opt.transform.rotation.x
        base_to_opt_pose.orientation.y = base_to_opt.transform.rotation.y
        base_to_opt_pose.orientation.z = base_to_opt.transform.rotation.z
        base_to_opt_pose.orientation.w = base_to_opt.transform.rotation.w
        
        base_to_opt_mat = pose_to_transformation_matrix(base_to_opt_pose)
        # print("rgb_to_cambase_mat", rgb_to_cambase_mat)

        block_to_opt_mat = pose_to_transformation_matrix(block_to_opt_pose)
        # print("panda_to_camrgb_mat", panda_to_camrgb_mat)

        base_to_block_mat = np.matmul(base_to_opt_mat, block_to_opt_mat)
        base_to_block_pose = transformation_matrix_to_pose(base_to_block_mat)

        base_to_block_pose_stamp = copy.deepcopy(opt_to_block_pose_stamped)
        base_to_block_pose_stamp.header.frame_id = "panda_link0"
        base_to_block_pose_stamp.pose = base_to_block_pose
        return base_to_block_pose_stamp
    except:
        print("tf lookup failed in block pose conversion")
        return None

def fit_plane_pcd(pcd: np.ndarray):
    "Fit plane using RANSAC on Nx3 pointcloud"
    def fit_plane(points):
        """Fits a plane to 3 points"""
        # compute plane normal
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        n = np.cross(v1, v2)
        n = n / np.linalg.norm(n)
        # compute d
        d = -np.dot(n, points[0])
        return np.hstack((n, d))
    
    def compute_inliers(plane, pcd, threshold):
        """Computes the inliers for a plane"""
        # plane dims (4,)
        # pcd dims (N,3)
        # threshold scalar

        # compute distances
        dist = np.abs(np.dot(pcd, plane[:3]) + plane[3])
        # compute inliers
        inliers = np.argwhere(dist < threshold).reshape(-1)
        return inliers
    
    num_itrs = 100
    threshold = 0.0005
    best_inliers = []
    best_plane = None
    for i in range(num_itrs):
        # randomly select 3 points
        idx = np.random.randint(0, pcd.shape[0], 3)
        points = pcd[idx, :]
        # fit plane
        plane = fit_plane(points)
        # compute inliers
        inliers = compute_inliers(plane, pcd, threshold)
        # update best plane
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = plane
    pcd_plane = pcd[best_inliers, :]
    return pcd_plane

def compute_pose(pcd_cropped:np.ndarray=None):
    """Computes the pose of the block"""
    pcd_cropped = pcd_cropped.reshape(-1,3)
    centroid = np.mean(pcd_cropped,axis=0)
    pcd_cropped = pcd_cropped - centroid
    cov = np.cov(pcd_cropped.T) #covariance matrix dims (3,3)
    u,s,v = np.linalg.svd(cov)

    # get 3rd column of u
    z_axis1 = u[:,2]
    z_axis2 = -u[:,2]

    #select z axis that is pointing downwards
    if(z_axis1[2]<0):
        z_axis = z_axis1
    else:
        z_axis = z_axis2

    #get y axis (its the x axis of the plane as we want to pickup along length of block)
    y_axis1 = u[:,0]
    y_axis2 = -u[:,0]

    # select x axis that is pointing towards global x axis
    if(y_axis1[0]>0):
        y_axis = y_axis1
    else:
        y_axis = y_axis2
    
    # get y axis
    x_axis = np.cross(y_axis,z_axis)
    rot_mat = np.vstack((x_axis,y_axis,z_axis)).T

    rot = R.from_matrix(rot_mat)
    quat = rot.as_quat()
    centroid[2] = centroid[2] - 0.015/2 # add half height of block
    
    pose_in_base = PoseStamped()
    pose_in_base.header.stamp = rospy.Time.now()
    pose_in_base.header.frame_id = "panda_link0"
    pose_in_base.pose.position.x = centroid[0]
    pose_in_base.pose.position.y = centroid[1]
    pose_in_base.pose.position.z = centroid[2]
    pose_in_base.pose.orientation.x = quat[0]
    pose_in_base.pose.orientation.y = quat[1]
    pose_in_base.pose.orientation.z = quat[2]
    pose_in_base.pose.orientation.w = quat[3]
    return pose_in_base

def compute_hover_pose(base_to_block_pose: PoseStamped=None):
    # input is pose in base frame
    # first compute opt_to_block_pose
    # translate y axis 5 cm, x axis -5
    # convert back to base frame
    try:
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        offset_mat = np.eye(4)
        offset_mat[0,3] = 0.05
        offset_mat[1,3] = 0
        
        base_to_block_mat = pose_to_transformation_matrix(base_to_block_pose.pose)
        base_to_block_mat_trans = np.matmul(base_to_block_mat, offset_mat)
        base_to_block_pose_trans = transformation_matrix_to_pose(base_to_block_mat_trans)

        # convert to PoseStamped
        base_to_block_pose_new_stamped = copy.deepcopy(base_to_block_pose)
        base_to_block_pose_new_stamped.pose = base_to_block_pose_trans
        return base_to_block_pose_new_stamped
    except:
        print("tf lookup failed in block pose conversion")
        return None

def compute_best_mask(mask_arr:np.ndarray=None,pointcloud=None):

    """Computes the best suited mask for grasping. Finds the most isolated block.
    Takes O(N^2) time.)"""

    dist=np.ones(mask_arr.shape[0])*1000000
    centroids=[]
    print(pointcloud.shape)
    for i in range(mask_arr.shape[0]):
        median = np.median(np.argwhere(mask_arr[i]>0).reshape(-1,2),axis=0).astype(np.int32)
        #reverse the order due to inconsistency in numpy and opencv
        centroids.append(pointcloud[median[0],median[1]])
    centroids = np.array(centroids)

    for i in range(mask_arr.shape[0]):
        for j in range(mask_arr.shape[0]):
            if(i!=j):
                curr_dist= np.linalg.norm(centroids[i]-centroids[j])
                if(curr_dist<dist[i]):
                    dist[i]=curr_dist
    best_mask_id = np.argmax(dist)
    best_mask = mask_arr[best_mask_id]
    assert best_mask.shape == mask_arr[0].shape, f"best mask shape={best_mask.shape} while mask_arr[0] shape={mask_arr[0].shape}"
    print("best_mask shape=",best_mask.shape)
    return best_mask, best_mask_id

def compute_best_mask_3d(mask_arr:np.ndarray=None,pointcloud=None):

    """Computes the best suited mask for grasping. Finds the most isolated block.
    Takes O(N^2) time.)"""

    pass

def compute_best_mask_2d(mask_arr:np.ndarray=None,pointcloud=None):
    """give the first mask that is almost a complete rectangle, find this using eigen values of the covariance matrix"""
    for mask_id in range(0, mask_arr.shape[0]):
        mask = mask_arr[mask_id]
        # get covariance matrix
        mask = mask.reshape(-1,2)
        mask = mask.astype(np.float32)
        mask = mask - np.mean(mask,axis=0)
        cov = np.cov(mask.T)
        u,s,v = np.linalg.svd(cov)
        print("Mask ID: ", mask_id, "Eigen Values: ", s, "Ratio: ", np.sqrt(s[0]/s[1]) )
    

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


def plot_pcd(points, centroid=None, angle=None):

    sphere=None
    mesh_frame=None

    if angle is not None:
        roll,pitch,yaw = angle
        #convert roll, pitch,yaw to rotation matrix
        r = R.from_euler('zyx', [roll, pitch, yaw], degrees=True)
        rot_matrix = np.array(r.as_matrix())
        #convert R to numpy array
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.025)
        mesh_frame.translate(centroid)
        mesh_frame.rotate(rot_matrix)

    if(centroid is not None):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.0025)
        sphere.paint_uniform_color([1, 0, 0])  # Set color to red
        sphere.translate(centroid)

    points = points.reshape((-1,3))
    plane_cloud = o3d.geometry.PointCloud()                 # creating point cloud of plane
    plane_cloud.points = o3d.utility.Vector3dVector(points)

    if(centroid is not None):
        if(angle is not None):
            o3d.visualization.draw_geometries([plane_cloud,sphere,mesh_frame])
        else:
            o3d.visualization.draw_geometries([plane_cloud,sphere])
    else:
        if(angle is not None):
            o3d.visualization.draw_geometries([plane_cloud,mesh_frame])
        else:
            o3d.visualization.draw_geometries([plane_cloud])

def plot(image,type="normal",title="Image"):
    if type=="colormap":
        image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03),  cv2.COLORMAP_HOT)

    elif(title=="HSV Mask"):
        cv2.imshow(title, image)
        cv2.waitKey(3)
        # cv2.createTrackbar('slider', title, 15, 70, hsv_tuning)

    cv2.imshow(title, image)
    cv2.waitKey(100)

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

def compute_hsv_mask(color_img,lower=None,upper=None):
    hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image,lower, upper)
    color_mask = cv2.bitwise_and(color_img, color_img, mask=mask)
    return color_mask,mask

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