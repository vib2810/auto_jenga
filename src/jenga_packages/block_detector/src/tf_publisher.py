#!/usr/bin/env python

import rospy
import rospkg
import yaml
import tf2_ros
import geometry_msgs.msg

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

    print(static_transformStamped)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_broadcaster.sendTransform(static_transformStamped)

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_node')

    # Load static transform parameters from YAML file
    rospack = rospkg.RosPack()
    static_tf_file = rospack.get_path('block_detector') + '/config/azure_cam.yaml'

    with open(static_tf_file, 'r') as f:
        static_tf_params = yaml.load(f, Loader=yaml.FullLoader)
    print(static_tf_params)
    # Publish static transform message
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        static_tf_broadcaster(static_tf_params)
        rate.sleep()

    
