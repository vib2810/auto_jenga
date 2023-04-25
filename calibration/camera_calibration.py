import socket
import time

import numpy as np
import rospy
from autolab_core import RigidTransform
from frankapy import FrankaArm
from geometry_msgs.msg import Pose
from move_arm import relative_translate_end_effector, rotate_end_effector
from utils import average_rigid_transforms, get_calib_file_path

CONFIG = {
    "calib_block_pre_inserted": True,
    "num_calib_snapshots": 10,
}

def transform_backward(avg_camera_global):
    import rospy
    import tf2_ros
    from tf import transformations
    from geometry_msgs.msg import TransformStamped
    import numpy as np
    from tf.transformations import quaternion_matrix


    # get ros transform between camera_base and rgb_camera_link using ros tf api
    # create tf subscriber
    rospy.init_node('tf_listener')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # get transform and store in output
    # rgb_to_base = tf_buffer.lookup_transform('camera_base', 'rgb_camera_link', rospy.Time(0), rospy.Duration(1.0))

    rgb_to_base = tf_buffer.lookup_transform('rgb_camera_link', 'camera_base', rospy.Time(0), rospy.Duration(1.0))


    rgb_to_base_mat = quaternion_matrix([rgb_to_base.transform.rotation.w,
                                        rgb_to_base.transform.rotation.x,
                                        rgb_to_base.transform.rotation.y,
                                        rgb_to_base.transform.rotation.z])
    rgb_to_base_mat[:3,3] = np.array([rgb_to_base.transform.translation.x, rgb_to_base.transform.translation.y, rgb_to_base.transform.translation.z])
    print("rgb_to_base_mat",rgb_to_base_mat)

    # comvert output to rpy
    output_rot = transformations.euler_from_quaternion([rgb_to_base.transform.rotation.x,rgb_to_base.transform.rotation.y,rgb_to_base.transform.rotation.z,rgb_to_base.transform.rotation.w])

    #create a transform orig from xyz and rpy values to publish with tf broadcaster
    panda_to_rgb = TransformStamped()
    panda_to_rgb.header.stamp = rospy.Time.now()
    panda_to_rgb.header.frame_id = "panda_link0"
    panda_to_rgb.child_frame_id = "rgb_camera_link"
    # values = [0.6528085542312886, 0.008211801262307983, 0.7219888127054556, 3.1177947993551447, 0.018598826321193695, -1.5788887028103056]
    values = [0.6410767589400161, 0.007342124308252686, 0.7266373291664254, -3.1253399618727586, 0.08398761063442337, -1.586073814121759]
    panda_to_rgb.transform.translation.x = values[0]
    panda_to_rgb.transform.translation.y = values[1]
    panda_to_rgb.transform.translation.z = values[2]
    yaw, pitch, roll = values[3], values[4], values[5]
    quat_orig = transformations.quaternion_from_euler(roll, pitch, yaw)
    panda_to_rgb.transform.rotation.x = quat_orig[0]
    panda_to_rgb.transform.rotation.y = quat_orig[1]
    panda_to_rgb.transform.rotation.z = quat_orig[2]
    panda_to_rgb.transform.rotation.w = quat_orig[3]

    panda_to_rgb_mat = quaternion_matrix([quat_orig[3], quat_orig[0], quat_orig[1], quat_orig[2]])
    panda_to_rgb_mat[:3,3] = np.array([panda_to_rgb.transform.translation.x, panda_to_rgb.transform.translation.y, panda_to_rgb.transform.translation.z])

    # apply output to orig
    # convert rotations to matrix

    print(panda_to_rgb_mat)

    final_tf = np.matmul(panda_to_rgb_mat, rgb_to_base_mat)

    print(transformations.euler_from_matrix(final_tf[:3,:3]))

    print("translations=",final_tf[:3,3])


    # panda_to_rgb_trans = np.array([panda_to_rgb.transform.translation.x, panda_to_rgb.transform.translation.y, panda_to_rgb.transform.translation.z])
    # rgb_to_base_trans = np.array([rgb_to_base.transform.translation.x, rgb_to_base.transform.translation.y, rgb_to_base.transform.translation.z])
    # panda_to_base_trans = panda_to_rgb_trans + rgb_to_base_trans

    # # panda_to_base_rot = panda_to_rgb_quat * rgb_to_base_quat
    # panda_to_rgb_quat = transformations.quaternion_from_euler(panda_to_rgb.transform.rotation.x,panda_to_rgb.transform.rotation.y,panda_to_rgb.transform.rotation.z)
    # rgb_to_base_quat = transformations.quaternion_from_euler(rgb_to_base.transform.rotation.x,rgb_to_base.transform.rotation.y,rgb_to_base.transform.rotation.z)
    # panda_to_base_quat = transformations.quaternion_multiply(rgb_to_base_quat, panda_to_rgb_quat)


    # # convert rot_combined to rpy
    # panda_to_base_rot = transformations.euler_from_quaternion(panda_to_base_quat, axes='sxyz')
    # print("panda_to_base_trans:", panda_to_base_trans)
    # print("combined rotation:", panda_to_base_rot)

    # # print in x y z y p r format
    # print("Values in xyz ypr format:")
    # print(panda_to_base_trans[0])
    # print(panda_to_base_trans[1])
    # print(panda_to_base_trans[2])
    # print(panda_to_base_rot[2])
    # print(panda_to_base_rot[1])
    # print(panda_to_base_rot[0])


def main(fa):
    # Reset joints
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
    # end_effector_pose = fa.get_pose()
    # Offset from end effector pose to the center of the aruco marker on the calibration block.
    offset_pose = RigidTransform(
        translation=np.array([0.0425, 0, -0.01]),
        rotation=np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),
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

    # Calibration usage:
    # aruco_global = camera_global*aruco_rt_pose

    # camera pose in global (robot_base frame)
    avg_camera_global.save(get_calib_file_path())
    cam_tf = Pose()
    cam_tf.position.x = avg_camera_global.translation[0]
    cam_tf.position.y = avg_camera_global.translation[1]
    cam_tf.position.z = avg_camera_global.translation[2]
    cam_tf.orientation.w = avg_camera_global.quaternion[0]
    cam_tf.orientation.x = avg_camera_global.quaternion[1]
    cam_tf.orientation.y = avg_camera_global.quaternion[2]
    cam_tf.orientation.z = avg_camera_global.quaternion[3]
    print("cam_tf Pose:\n {}".format(cam_tf))
    
    calibrated_tf = transform_backward(avg_camera_global)

if __name__ == "__main__":
    fa = FrankaArm()
    main(fa)
