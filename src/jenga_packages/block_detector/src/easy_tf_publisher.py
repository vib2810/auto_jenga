#!/usr/bin/env python

import rospy
import rospkg
import yaml
import tf2_ros
import geometry_msgs.msg
import numpy as np
import scipy.spatial.transform as spt
from geometry_msgs.msg import Pose

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

def transform_backward(panda_to_camrgb_pose):
    # get ros transform between camera_base and rgb_camera_link using ros tf api
    # create tf subscriber
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # rgb_to_cambase = tf_buffer.lookup_transform('camera_base', 'rgb_camera_link', rospy.Time(0), rospy.Duration(1.0))
    rgb_to_cambase = tf_buffer.lookup_transform('rgb_camera_link', 'camera_base', rospy.Time(0), rospy.Duration(1.0))
    rgb_to_cambase_pose = Pose()
    rgb_to_cambase_pose.position.x = rgb_to_cambase.transform.translation.x
    rgb_to_cambase_pose.position.y = rgb_to_cambase.transform.translation.y
    rgb_to_cambase_pose.position.z = rgb_to_cambase.transform.translation.z
    rgb_to_cambase_pose.orientation.x = rgb_to_cambase.transform.rotation.x
    rgb_to_cambase_pose.orientation.y = rgb_to_cambase.transform.rotation.y
    rgb_to_cambase_pose.orientation.z = rgb_to_cambase.transform.rotation.z
    rgb_to_cambase_pose.orientation.w = rgb_to_cambase.transform.rotation.w
    
    rgb_to_cambase_mat = pose_to_transformation_matrix(rgb_to_cambase_pose)
    print("rgb_to_cambase_mat", rgb_to_cambase_mat)

    panda_to_camrgb_mat = pose_to_transformation_matrix(panda_to_camrgb_pose)
    print("panda_to_camrgb_mat", panda_to_camrgb_mat)

    panda_to_cambase_mat = np.matmul(panda_to_camrgb_mat, rgb_to_cambase_mat)
    panda_to_cambase_pose = transformation_matrix_to_pose(panda_to_cambase_mat)
    return panda_to_cambase_pose

def static_tf_broadcaster(static_tf_params: Pose):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "panda_link0"
    static_transformStamped.child_frame_id =  "/camera_base"
    static_transformStamped.transform.translation.x = static_tf_params.position.x
    static_transformStamped.transform.translation.y = static_tf_params.position.y
    static_transformStamped.transform.translation.z = static_tf_params.position.z
    static_transformStamped.transform.rotation.x = static_tf_params.orientation.x
    static_transformStamped.transform.rotation.y = static_tf_params.orientation.y
    static_transformStamped.transform.rotation.z = static_tf_params.orientation.z
    static_transformStamped.transform.rotation.w = static_tf_params.orientation.w

    print(static_transformStamped)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_broadcaster.sendTransform(static_transformStamped)

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_node')

    # Load static transform parameters from YAML file
    rospack = rospkg.RosPack()
    static_tf_file = rospack.get_path('block_detector') + '/config/azure_easy.yaml'

    with open(static_tf_file, 'r') as f:
        static_tf_params = yaml.load(f, Loader=yaml.FullLoader)
    print(static_tf_params)
    static_tf_pose = Pose()
    static_tf_pose.position.x = static_tf_params["pose"]['translation']['x']
    static_tf_pose.position.y = static_tf_params["pose"]['translation']['y']
    static_tf_pose.position.z = static_tf_params["pose"]['translation']['z']
    static_tf_pose.orientation.x = static_tf_params["pose"]['rotation']['x']
    static_tf_pose.orientation.y = static_tf_params["pose"]['rotation']['y']
    static_tf_pose.orientation.z = static_tf_params["pose"]['rotation']['z']
    static_tf_pose.orientation.w = static_tf_params["pose"]['rotation']['w']
    corrected_static_tf_pose = transform_backward(static_tf_pose)
    
    # Publish static transform message
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        static_tf_broadcaster(corrected_static_tf_pose)
        rate.sleep()

    
