import socket
import time

import numpy as np
import rospy
from autolab_core import RigidTransform
import sys
sys.path.append("/home/ros_ws/src/git_packages/frankapy")
from frankapy import FrankaArm
from geometry_msgs.msg import Pose
import rospy
import tf2_ros
from tf import transformations
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf.transformations import quaternion_matrix
import geometry_msgs.msg
import scipy.spatial.transform as spt
import yaml
import copy
import rospkg

CONFIG = {
    "calib_block_pre_inserted": True,
    "num_calib_snapshots": 10,
}

def get_current_translation(fa):
    return fa.get_pose().translation


def get_current_rotation(fa):
    return fa.get_pose().rotation


def move_end_effector(
    fa, rot, trans, duration=None, use_impedance=True, cartesian_impedances=None, verbose=True
):

    des_pose = RigidTransform(
        rotation=rot, translation=trans, from_frame="franka_tool", to_frame="world"
    )
    fa.goto_pose(
        des_pose,
        duration=duration,
        use_impedance=use_impedance,
        cartesian_impedances=cartesian_impedances,
    )


def relative_translate_end_effector(
    fa,
    x_offset=0.0,
    y_offset=0.0,
    z_offset=0.0,
    duration=None,
    use_impedance=True,
    cartesian_impedances=None,
    verbose=False,
):
    new_trans = copy.deepcopy(get_current_translation(fa))
    new_trans[0] += x_offset
    new_trans[1] += y_offset
    new_trans[2] += z_offset

    move_end_effector(
        fa,
        rot=get_current_rotation(fa),
        trans=new_trans,
        duration=duration,
        use_impedance=use_impedance,
        cartesian_impedances=cartesian_impedances,
        verbose=verbose,
    )


def rotate_end_effector(
    fa,
    rot,
    duration=None,
    use_impedance=True,
    cartesian_impedances=None,
    verbose=False,
):
    move_end_effector(
        fa,
        rot=rot,
        trans=get_current_translation(fa),
        duration=duration,
        use_impedance=use_impedance,
        cartesian_impedances=cartesian_impedances,
        verbose=verbose,
    )

def average_rigid_transforms(rigid_transforms):
    num_transforms = len(rigid_transforms)
    if num_transforms == 0:
        return None

    # Define lists to store the translation and rotation components of each transform
    t_vals = []
    R_vals = []
    from_frames = set()
    to_frames = set()

    # Iterate over each transform and append its components to the corresponding list
    for transform in rigid_transforms:
        t_vals.append(transform.translation)
        R_vals.append(transform.rotation)
        from_frames.add(transform.from_frame)
        to_frames.add(transform.to_frame)
    
    assert len(from_frames) == 1
    assert len(to_frames) == 1

    # Compute the average of the translation and rotation components
    avg_t = np.mean(t_vals, axis=0)
    avg_R = np.mean(R_vals, axis=0)

    # Create a new RigidTransform with the averaged translation and rotation components
    avg_transform = RigidTransform(avg_R, avg_t, from_frame=list(from_frames)[0], to_frame=list(to_frames)[0])

    return avg_transform

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

    rgb_to_cambase = tf_buffer.lookup_transform('camera_base', 'rgb_camera_link', rospy.Time(0), rospy.Duration(1.0))
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

def static_tf_broadcaster(static_tf_params):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "panda_link0"
    static_transformStamped.child_frame_id =  "/camera_base"
    static_transformStamped.transform.translation.x = static_tf_params["pose"]['position']['x']
    static_transformStamped.transform.translation.y = static_tf_params["pose"]['position']['y']
    static_transformStamped.transform.translation.z = static_tf_params["pose"]['position']['z']
    static_transformStamped.transform.rotation.x = static_tf_params["pose"]['orientation']['x']
    static_transformStamped.transform.rotation.y = static_tf_params["pose"]['orientation']['y']
    static_transformStamped.transform.rotation.z = static_tf_params["pose"]['orientation']['z']
    static_transformStamped.transform.rotation.w = static_tf_params["pose"]['orientation']['w']

    # print("Publishing ")
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_broadcaster.sendTransform(static_transformStamped)

if __name__ == "__main__":
    rospy.init_node('tf_listener')
    fa = FrankaArm(init_node=False)
    # fa.reset_joints()

    # if not CONFIG["calib_block_pre_inserted"]:
    #     # Open gripper to insert calibration block with aruco marker.
    #     fa.open_gripperrobot_base()
    #     # You have 3 seconds to insert the calibration block.
    #     time.sleep(2)
    #     # Close gripper to hold calibration block.
    #     fa.close_gripper()

    # # Move gripper to a pose with good visibility in the camera's FOV.
    # rot = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    # relative_translate_end_effector(fa, x_offset=0.25, duration=3.0)
    # rotate_end_effector(fa, rot, duration=3.0)

    # Get end effector pose from franka arm.
    end_effector_pose = fa.get_pose()
    # Offset from end effector pose to the center of the aruco marker on the calibration block.
    # convert euler to rotation matrix using spt
    rot_mat = spt.Rotation.from_euler('zyx', [0, 180, 180], degrees=True).as_matrix()
    offset_pose = RigidTransform(
        # translation=np.array([0.015, 0.103, 0.0425]),
        translation=np.array([0.3, -0.3, -0.35]),
        rotation=rot_mat,
        from_frame="aruco_pose",
        to_frame="franka_tool",
    )

    # Capture marker pose at multiple timestamps and average these to reduce effect of outliers.
    camera_global_snapshots = []
    for i in range(CONFIG["num_calib_snapshots"]):
        aruco_pose = rospy.wait_for_message(
            "/aruco_simple/pose", Pose, timeout=5
        )
        aruco_rt_pose = RigidTransform.from_pose_msg(
            aruco_pose, from_frame="aruco_pose"
        )
        camera_global = (
            end_effector_pose * offset_pose * aruco_rt_pose.inverse()
        )
        print("camera_global: {}".format(camera_global))
        camera_global_snapshots.append(camera_global)

    avg_camera_global = average_rigid_transforms(camera_global_snapshots)
    print("avg_camera_global: {}".format(avg_camera_global))

    panda_to_camrgb_pose = Pose()
    panda_to_camrgb_pose.position.x = avg_camera_global.translation[0]
    panda_to_camrgb_pose.position.y = avg_camera_global.translation[1]
    panda_to_camrgb_pose.position.z = avg_camera_global.translation[2]
    panda_to_camrgb_pose.orientation.w = avg_camera_global.quaternion[0]
    panda_to_camrgb_pose.orientation.x = avg_camera_global.quaternion[1]
    panda_to_camrgb_pose.orientation.y = avg_camera_global.quaternion[2]
    panda_to_camrgb_pose.orientation.z = avg_camera_global.quaternion[3]
    print("panda_to_camrgb_pose:\n {}".format(panda_to_camrgb_pose))

    # transform from camrgb to cambase
    panda_to_cambase_pose = transform_backward(panda_to_camrgb_pose)
    # save panda_to_cambase_pose to yaml file
    pose_dict = {'pose': {'position': {'x': float(panda_to_cambase_pose.position.x),
                                                    'y': float(panda_to_cambase_pose.position.y),
                                                    'z': float(panda_to_cambase_pose.position.z)},
                        'orientation': {'x': float(panda_to_cambase_pose.orientation.x),
                                        'y': float(panda_to_cambase_pose.orientation.y),
                                        'z': float(panda_to_cambase_pose.orientation.z),
                                        'w': float(panda_to_cambase_pose.orientation.w)}}}
    # Save YAML to file
    with open('/home/ros_ws/src/jenga_packages/block_detector/config/azure_cam.yaml', 'w') as f:
        yaml.dump(pose_dict, f)
    
     # Load static transform parameters from YAML file
    rospack = rospkg.RosPack()
    static_tf_file = rospack.get_path('block_detector') + '/config/azure_cam.yaml'

    with open(static_tf_file, 'r') as f:
        static_tf_params = yaml.load(f, Loader=yaml.FullLoader)
    print(static_tf_params)
    # Publish static transform message
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        static_tf_broadcaster(static_tf_params)
        rate.sleep()

    rospy.spin()

