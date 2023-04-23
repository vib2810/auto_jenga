#!/usr/bin/env python

import rospy
import actionlib
from jenga_msgs.msg import GetBlocksAction, GetBlocksResult
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from rospy.numpy_msg import numpy_msg
import message_filters

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

    # def hsv_tuning(self,lb_h):

    #   imagecpy = self.image.copy()
    #   print(imagecpy.shape)
    #   self.lower_hsv2 = (5, 25, 20)
    #   self.upper_hsv2 = (lb_h, 250, 255)
    #   mask = self.compute_hsv_mask(imagecpy,self.lower_hsv2,self.upper_hsv)
    #   cv2.imshow(self.title, mask)
    #   cv2.waitKey(100)


    def plot(self,image,type="normal",title="Image"):
        if type=="colormap":
            image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03),  cv2.COLORMAP_HOT)

        if(title=="HSV Mask"):
            self.title = title
        cv2.imshow(title, image)
        cv2.waitKey(3)
        # cv2.createTrackbar('slider', title, 15, 70, self.hsv_tuning)

        cv2.imshow(title, image)
        cv2.waitKey(3)

    def opening_morphology(self,image,kernel_size=5):
        kernel = np.ones((kernel_size,kernel_size),np.uint8)
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        return opening

    def erode(self,mask,kernel):
        kernel = np.ones((kernel,kernel),np.uint8)
        erosion = cv2.erode(mask,kernel)
        return erosion
        
    def preprocess_image(self,color_img,depth_img):

        #crop half image
        color_img = color_img[200:720, 200:640].copy()
        depth_img = depth_img[200:720, 200:640].copy()
        
        #filter erroenous depth values
        depth_img[depth_img<self.depth_threshold]=depth_img.max()

        #Normalize depth image
        # depth_img = (depth_img/depth_img.max())*255
        # depth_img = depth_img.astype(np.uint8)

        # ret3,depth_img = cv2.threshold(depth_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        depth_mask = np.zeros_like(depth_img)
        min_depth = depth_img.min()
        depth_mask[np.logical_and(depth_img>min_depth,depth_img<min_depth+self.block_height)]=255
        depth_mask=depth_mask.astype(np.uint8)

        depth_mask = self.opening_morphology(depth_mask)
        
        return color_img, depth_img,depth_mask

    def compute_hsv_mask(self,color_img,lower,upper):
        hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        mask = mask = cv2.inRange(hsv_image,lower, upper)
        return mask

    def get_contours(self,mask):

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        # Define a minimum contour area threshold (change as necessary)
        min_contour_area = self.mask_pixels/1.5
        # Filter the contours based on their size
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_contour_area]


        return filtered_contours

    def draw_countours(self,image,contours,box:bool=False):
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

    def chatgptied(self,contours, image):

        for contour in contours:
            remaining_points = np.squeeze(contour)
        while len(remaining_points) >= 300: # at least 5 points needed for fitting an ellipse
            # Fit ellipse using cv2.fitEllipse()
            ellipse = cv2.fitEllipse(remaining_points)
            
            # Draw ellipse on original image
            cv2.ellipse(image, ellipse, (255, 255, 255), 2)

            print("hey")
            
            # Remove inliers from remaining points
            distances = cv2.pointPolygonTest(contour, (ellipse[0][0], ellipse[0][1]), True)
            inliers = np.where(distances < ellipse[1][0] / 2)[0]
            remaining_points = np.delete(remaining_points, inliers, axis=0)

        # Display image with ellipses
        cv2.imshow('Ellipses', image)
        cv2.waitKey(3)

    def ransac_rect(self,image,color, num_iterations=100, min_num_inliers=1000, inlier_threshold=5):
        """
        Uses RANSAC to fit a rectangle to an image.

        :param image: The input image.
        :param num_iterations: The maximum number of RANSAC iterations.
        :param min_num_inliers: The minimum number of inliers required to fit a rectangle.
        :param inlier_threshold: The maximum distance between a point and the rectangle for it to be considered an inlier.
        :return: The best rectangle found using RANSAC.
        """
        height, width = image.shape[:2]
        best_rect = None
        max_num_inliers = 0
        # get img_points tuple of x,y where mask is 255
        img_points = np.where(image==255)
        # convert to Nx2 array
        img_points = np.column_stack((img_points[1],img_points[0]))
        # print("img_points",img_points.shape)    

        for i in range(num_iterations):
            # Randomly sample 4 points to define a candidate rectangle
            idx = np.random.choice(height * width, 4, replace=False)
            points = np.column_stack(np.unravel_index(idx, (height, width)))
            # print("points",points)

            # rect = cv2.minAreaRect(points)
            # print("rect",rect)
            # print(i)

            rect_array = np.array(points)
            rect_array = np.expand_dims(rect_array,1)
            # print(rect_array)
            # Find inliers by calculating distance from each point to the rectangle
            distances = []
            for k in range(len(img_points)):
                dist = cv2.pointPolygonTest(rect_array, (img_points[k,0], img_points[k,1]), True)
                distances.append(dist)
                distances = np.array(distances)
                inliers = np.where(np.abs(distances) < inlier_threshold)[0]
                inliers = inliers.astype(np.int32)
            # print(inliers)

            print(i,len(inliers))

            if len(inliers) >= min_num_inliers:
                # Re-fit rectangle to all inliers
                inlier_points = img_points[inliers]
                rect = cv2.minAreaRect(inlier_points)

                # Store if best so far
                num_inliers = len(inliers)
                if num_inliers > max_num_inliers:
                    max_num_inliers = num_inliers
                    best_rect = rect_array

        #plot best_rect on image
        # best_rect is a 4x2 array of x,y points
        image = cv2.drawContours(color, [best_rect], 0, (0,0,255), 2)

        self.plot(image,title="RANSAC")


        return best_rect

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
        hsv_mask = self.compute_hsv_mask(color_img,self.lower_hsv,self.upper_hsv)
        # self.plot(hsv_mask,title="HSV Mask")

        #draw contours
        depth_mask = self.erode(depth_mask,9)

        #draw harris corners
        self.draw_harris_corners(color_img, depth_mask)

        # print("count white pixels=",(depth_mask.sum()/255)//4)
        contours = self.get_contours(depth_mask)

        #rectanlges using chatgpt
        self.ransac_rect(depth_mask,color_img)

        color_mask = self.draw_countours(color_img,contours,box=True)
        self.plot(color_mask,title="Color Mask")

        #apply K means on depth mask
        self.compute_k_means(depth_mask,color_img)

        rate = rospy.Rate(10) # 10hz
        rate.sleep()

class BlockDetector(object):
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("get_blocks", GetBlocksAction, self.execute_action, False)
        self.action_server.start()

        self.bridge = CvBridge()
        
        #write code for message filters subscriber for depth and color images
        self.color_sub=message_filters.Subscriber("/rgb/image_raw", Image)
        self.depth_sub=message_filters.Subscriber("/depth_to_rgb/image_raw", Image)

        rospy.loginfo(rospy.get_caller_id() + "Subscribed to topics!!!")

        ts=message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],10,1)
        ts.registerCallback(self.inference) 

    def inference(self, color, depth):
        rospy.loginfo("Images received!!!")
        
        try:
            color_img = self.bridge.imgmsg_to_cv2(color, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            
        except CvBridgeError as e:
            print(e)

        self.color_img=color_img
        self.depth_img=depth_img
        # # show color_img
        # cv2.imshow("color_img",color_img)
        # cv2.waitKey(1)


    def execute_action(self, goal):
        rospy.loginfo("Block Detector Action Called!!!")
        result = GetBlocksResult()
        feedback = GetBlocksAction.Feedback()
        # feedback.feedback_progress = 0.5
        # self.action_server.publish_feedback(feedback)
        # RANSAC to detect block in image
        detected_block = Image_Proc.ransac_rect(self.color_img, self.depth_img)

        #publish a Pose message of the block 6D pose
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = "panda_link0"
        result.pose.position.x = detected_block[0][0]
        result.pose.position.y = detected_block[0][1]
        result.pose.position.z = detected_block[0][2]
        result.pose.orientation.x = detected_block[1][0]
        result.pose.orientation.y = detected_block[1][1]
        result.pose.orientation.z = detected_block[1][2]
        result.pose.orientation.w = detected_block[1][3]

        self.action_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("block_detector")
    block_detector = BlockDetector()
    rospy.spin()