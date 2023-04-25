import rospy
import tf2_ros
from tf import transformations
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf.transformations import quaternion_matrix
from transformations import quaternion_matrix


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
# read transform from tf file
with open("calib/iam-luisa-kinect-transform.tf", "r") as f:
    lines = f.readlines()

x, y, z, qx, qy, qz, qw = [float(i) for i in lines[0].split()]
tf_file = open("panda_to_rgb.txt", "r")
panda_to_rgb = TransformStamped()

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

