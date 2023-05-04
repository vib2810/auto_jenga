#!/usr/bin/env python
import sys
import rospy
# import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from move_it_block_pose_planner.srv import *
import quaternion
from autolab_core import RigidTransform
# 75mm length, 15mm height, 25 mm width


import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
import scipy.spatial.transform as spt

sys.path.append("/home/ros_ws/src/git_packages/frankapy")
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
import copy
from jenga_msgs.msg import GetBlocksAction, GetBlocksResult, GetBlocksGoal
import actionlib
sys.path.append("/home/ros_ws/src/jenga_packages/block_detector/src/")
from utils import compute_hover_pose

class moveit_planner():
    def __init__(self) -> None: #None means no return value
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id("RRTstarkConfigDefault")
        self.group.set_end_effector_link("panda_hand")
        self.obs_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

        # used to visualize the planned path
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        planning_frame = self.group.get_planning_frame()
        eef_link = self.group.get_end_effector_link()

        print("---------Moveit Planner Class Initialized---------")
        print("Planning frame: ", planning_frame)
        print("End effector: ", eef_link)
        print("Robot Groups:", self.robot.get_group_names())

        # frankapy 
        self.pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        self.fa = FrankaArm(init_node = False)
    
    # Utility Functions
    def print_robot_state(self):
        print("Joint Values:\n", self.group.get_current_joint_values())
        print("Pose Values (panda_hand):\n", self.group.get_current_pose())
        print("Pose Values (panda_end_effector):\nNOTE: Quaternion is (w,x,y,z)\n", self.fa.get_pose())
     
    def reset_joints(self):
        self.fa.reset_joints()

    def goto_joint(self, joint_goal):
        self.fa.goto_joints(joint_goal, duration=5, dynamic=True, buffer_time=10)
    
    def get_plan_given_pose(self, pose_goal: geometry_msgs.msg.Pose):
        """
        Plans a trajectory given a tool pose goal
        Returns joint_values 
        joint_values: numpy array of shape (N x 7)
        """
        output = self.group.plan(pose_goal)
        plan = output[1]
        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values
    
    def get_plan_given_joint(self, joint_goal_list):
        """
        Plans a trajectory given a joint goal
        Returns joint_values and moveit plan
        joint_values: numpy array of shape (N x 7)
        plan: moveit_msgs.msg.RobotTrajectory object
        """
        joint_goal = sensor_msgs.msg.JointState()
        joint_goal.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        joint_goal.position = joint_goal_list

        output = self.group.plan(joint_goal)
        plan = output[1]
        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values, plan

    def execute_plan(self, joints_traj, speed = 1):
        """
        joints_traj shape: (N x 7)
        """
        # interpolate the trajectory
        num_interp_slow = int(50/speed) # number of points to interpolate for the start and end of the trajectory
        num_interp = int(20/speed) # number of points to interpolate for the middle part of the trajectory
        interpolated_traj = []
        t_linear = np.linspace(1/num_interp, 1, num_interp)
        t_slow = np.linspace(1/num_interp_slow, 1, num_interp_slow)
        t_ramp_up = t_slow**2
        t_ramp_down = 1 - (1-t_slow)**2

        interpolated_traj.append(joints_traj[0,:])
        for t_i in range(len(t_ramp_up)):
            dt = t_ramp_up[t_i]
            interp_traj_i = joints_traj[1,:]*dt + joints_traj[0,:]*(1-dt)
            interpolated_traj.append(interp_traj_i)
            
        for i in range(2, joints_traj.shape[0]-1):
            for t_i in range(len(t_linear)):
                dt = t_linear[t_i]
                interp_traj_i = joints_traj[i,:]*dt + joints_traj[i-1,:]*(1-dt)
                interpolated_traj.append(interp_traj_i)

        for t_i in range(len(t_ramp_down)):
            dt = t_ramp_down[t_i]
            interp_traj_i = joints_traj[-1,:]*dt + joints_traj[-2,:]*(1-dt)
            interpolated_traj.append(interp_traj_i)

        interpolated_traj = np.array(interpolated_traj)

        print('Executing joints trajectory of shape: ', interpolated_traj.shape)

        rate = rospy.Rate(50)
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(interpolated_traj[1], duration=5, dynamic=True, buffer_time=30, ignore_virtual_walls=True)
        init_time = rospy.Time.now().to_time()
        for i in range(2, interpolated_traj.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=interpolated_traj[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            self.pub.publish(ros_msg)
            rate.sleep()

        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
        self.pub.publish(ros_msg)
        self.fa.stop_skill()
    
    def get_straight_plan_given_pose(self, pose_goal: geometry_msgs.msg.Pose):
        """
        pose_goal: geometry_msgs.msg.Pose
        Plans a trajectory given a tool pose goal
        Returns joint_values
        joint_values: numpy array of shape (N x 7)
        """
        waypoints = []
        waypoints.append(copy.deepcopy(pose_goal))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values
    
    # Test Functions
    def unit_test_joint(self, execute = False, guided = False):
        """
        Unit test for joint trajectory planning
        Resets to home and plans to the joint goal
        Displays the planned path to a fixed joint goal on rviz

        Parameters
        ----------
        execute: bool
            If True, executes the planned trajectory
        guided: bool
            If True, runs guide mode where the user can move the robot to a desired joint goal
            Else, uses a fixed joint goal
        """
        if guided:
            print("Running Guide Mode, Move Robot to Desired Pose")
            self.fa.run_guide_mode(10, block=True)
            self.fa.stop_skill()
            joint_goal_list = self.fa.get_joints()
            print("Joint Goal: ", joint_goal_list)
        else:
            # A random joint goal
            joint_goal_list = [6.14813255e-02 ,4.11382927e-01, 6.80023936e-02,-2.09547337e+00,-2.06094866e-03,2.56799173e+00,  9.20088362e-01]
        print("Resetting Joints")
        self.fa.reset_joints()
        plan_joint_vals, plan_joint = self.get_plan_given_joint(joint_goal_list)
        print("Planned Path Shape: ", plan_joint_vals.shape)
        if execute: 
            print("Executing Plan")
            self.execute_plan(plan_joint_vals)
    
    def unit_test_pose(self, execute = False, guided = False):
        """
        Unit test for pose trajectory planning
        Resets to home and plans to the pose goal
        Displays the planned path to a fixed pose goal on rviz

        Parameters
        ----------
        execute: bool
            If True, executes the planned trajectory
        guided: bool
            If True, runs guide mode where the user can move the robot to a desired pose goal
            Else, uses a fixed pose goal
        """
        print("Unit Test for Tool Pose Trajectory Planning")
        if guided:
            print("Running Guide Mode, Move Robot to Desired Pose")
            self.fa.run_guide_mode(10, block=True)
            self.fa.stop_skill()
            pose_goal_fa = self.fa.get_pose()
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = pose_goal_fa.translation[0]
            pose_goal.position.y = pose_goal_fa.translation[1]
            pose_goal.position.z = pose_goal_fa.translation[2]
            pose_goal.orientation.w = pose_goal_fa.quaternion[0]
            pose_goal.orientation.x = pose_goal_fa.quaternion[1]
            pose_goal.orientation.y = pose_goal_fa.quaternion[2]
            pose_goal.orientation.z = pose_goal_fa.quaternion[3]
        else:
            pose_goal = geometry_msgs.msg.Pose()
            # a random test pose
            pose_goal.position.x = 0.5843781940153249
            pose_goal.position.y = 0.05791107711908864
            pose_goal.position.z = 0.23098061041636195
            pose_goal.orientation.x = -0.9186984147774666
            pose_goal.orientation.y = 0.3942492534293267
            pose_goal.orientation.z = -0.012441904611284204 
            pose_goal.orientation.w = 0.020126567105018894
        
        # Convert to moveit pose
        pose_goal = self.get_moveit_pose_given_frankapy_pose(pose_goal)
        print("Pose Goal: ", pose_goal)
        print("Resetting Joints")
        self.fa.reset_joints()
        plan_pose = self.get_plan_given_pose(pose_goal)
        print("Planned Path Shape: ", plan_pose.shape)
        if execute:
            print("Executing Plan")
            self.execute_plan(plan_pose)

    def get_moveit_pose_given_frankapy_pose(self, pose):
        """
        Converts a frankapy pose (in panda_end_effector frame) to a moveit pose (in panda_hand frame) 
        by adding a 10 cm offset to z direction
        """
        transform_mat =  np.array([[1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,1,-0.1034],
                                   [0,0,0,1]])
        pose_mat = self.pose_to_transformation_matrix(pose)
        transformed = pose_mat @ transform_mat
        pose_goal = self.transformation_matrix_to_pose(transformed)
        return pose_goal
    
    def pose_to_transformation_matrix(self, pose):
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

    def transformation_matrix_to_pose(self, trans_mat):   
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

    def add_box(self, name, pose: geometry_msgs.msg.PoseStamped(), size):
        """
        Adds a collision box to the planning scene

        Parameters
        ----------
        name : str
            Name of the box
        pose : geometry_msgs.msg.PoseStamped
            Pose of the box (Centroid and Orientation)
        size : list
            Size of the box in x, y, z  
        """
        self.scene.add_box(name, pose, size)

    def remove_box(self, name):
        self.scene.remove_world_object(name)

def goto_pose_moveit(pose_goal: PoseStamped, speed = 1):
    "pose_goal in frame franka_end_effector (frankapy)"
    pose_goal = moveit_handler.get_moveit_pose_given_frankapy_pose(pose_goal.pose)
    plan = moveit_handler.get_straight_plan_given_pose(pose_goal)
    moveit_handler.execute_plan(plan, speed) 

def goto_pose_frankapy(pose_goal: PoseStamped):
    np_quat = quaternion.as_quat_array(np.array([pose_goal.pose.orientation.w, pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]))
    local_transl = np.array([pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z])
    local_pose_to_set = RigidTransform(rotation=quaternion.as_rotation_matrix(np_quat), translation = local_transl, from_frame="franka_tool", to_frame="world")
    moveit_handler.fa.goto_pose(local_pose_to_set, use_impedance=False, duration=7)

def hover_block(req_pose, dist=0.1, speed = 1):
    global moveit_handler
    hover_pose = copy.deepcopy(req_pose)
    hover_pose.pose.position.z = hover_pose.pose.position.z + dist

    goto_pose_moveit(hover_pose, speed)

    # hover_pose_quat = quaternion.as_quat_array(np.array([hover_pose.pose.orientation.w, hover_pose.pose.orientation.x, hover_pose.pose.orientation.y, hover_pose.pose.orientation.z]))
    # hover_pose_trans = np.array([hover_pose.pose.position.x, hover_pose.pose.position.y, hover_pose.pose.position.z])
    # hover_pose_fpy = RigidTransform(rotation=quaternion.as_rotation_matrix(hover_pose_quat), translation=hover_pose_trans, from_frame="franka_tool", to_frame="world")
    # moveit_handler.fa.goto_pose(hover_pose_fpy, duration=5)
    return True

def pickup_block(req_pose, speed = 1):
    global moveit_handler

    # moveit_handler.add_box(name="pickup_block", pose=req_pose, size=[0.02, 0.065, 0.015])
    goto_pose_moveit(req_pose, speed)

    # req_pose_quat = quaternion.as_quat_array(np.array([req_pose.pose.orientation.w, req_pose.pose.orientation.x, req_pose.pose.orientation.y, req_pose.pose.orientation.z]))
    # req_pose_trans = np.array([req_pose.pose.position.x, req_pose.pose.position.y, req_pose.pose.position.z])
    # req_pose_fpy = RigidTransform(rotation = quaternion.as_rotation_matrix(req_pose_quat), translation= req_pose_trans, from_frame="franka_tool", to_frame="world")
    # moveit_handler.fa.goto_pose(req_pose_fpy, duration=5)

    moveit_handler.fa.close_gripper()
    return True

def get_posestamped(translation, orientation):
    ret = PoseStamped()
    ret.header.frame_id = "panda_link0"
    ret.pose.position.x = translation[0]
    ret.pose.position.y = translation[1]
    ret.pose.position.z = translation[2]
    ret.pose.orientation.w = orientation[0]
    ret.pose.orientation.x = orientation[1]
    ret.pose.orientation.y = orientation[2]
    ret.pose.orientation.z = orientation[3]
    return ret

def drop_block_moveit(block_id, layer_id):
    global moveit_handler

    # Hard coded because staging area - need to update when testing with actual limits
    middle_block_pose_even ={'orientation': np.array([0.0, 1.0, 0.0, 0.0]), # w, x, y, z
                        'translation': np.array([0.5, 0, 0])}
    middle_block_pose_odd = {'orientation': np.array([0, 0.707, 0.707, 0.0]), # w, x, y, z
                        'translation': np.array([0.5, 0, 0])}
    
    drop_pose_quat = quaternion.as_quat_array(middle_block_pose_even['orientation'])
    if(layer_id%2 == 0):
        drop_pose_quat_local = middle_block_pose_even['orientation']
        drop_pose_transl = middle_block_pose_even['translation'] + (1 - block_id%3)*np.array([0.025, 0, 0])
        pose_quat_squeeze = middle_block_pose_odd['orientation']
    else:
        drop_pose_quat_local = middle_block_pose_odd['orientation']
        drop_pose_transl = middle_block_pose_odd['translation'] + (1 - block_id%3)*np.array([0, 0.026, 0])
        pose_quat_squeeze = middle_block_pose_even['orientation']
    # pose_to_drop = RigidTransform(rotation=quaternion.as_rotation_matrix(drop_pose_quat), translation = drop_pose_transl, from_frame="franka_tool", to_frame="world")
    
    drop_pose_transl = drop_pose_transl + (2*layer_id+1)*np.array([0,0,0.0075]) + np.array([0,0,0.003])

    # fa.goto_pose(pose_to_drop, use_impedance=False, duration=15)
    pose_to_drop = PoseStamped()
    pose_to_drop.header.frame_id = "panda_link0"
    pose_to_drop.pose.orientation.w = drop_pose_quat_local[0]
    pose_to_drop.pose.orientation.x = drop_pose_quat_local[1]
    pose_to_drop.pose.orientation.y = drop_pose_quat_local[2]
    pose_to_drop.pose.orientation.z = drop_pose_quat_local[3]
    pose_to_drop.pose.position.x = drop_pose_transl[0]
    pose_to_drop.pose.position.y = drop_pose_transl[1]
    pose_to_drop.pose.position.z = drop_pose_transl[2]

    hover_block(pose_to_drop, 0.1, speed = 2.5) 
    goto_pose_moveit(pose_to_drop, speed = 1.5)

    moveit_handler.fa.open_gripper()
    hover_block(pose_to_drop, 0.1, speed = 2.5) # hover above the tower
    
    if(block_id == 2):
        pose_to_drop_squeeze = copy.deepcopy(pose_to_drop)
        pose_to_drop_squeeze_transl = middle_block_pose_even['translation'] + (2*layer_id+1)*np.array([0,0,0.0075])

        pose_to_drop_squeeze.pose.position.x = pose_to_drop_squeeze_transl[0]
        pose_to_drop_squeeze.pose.position.y = pose_to_drop_squeeze_transl[1]
        pose_to_drop_squeeze.pose.position.z = pose_to_drop_squeeze_transl[2]
        pose_to_drop_squeeze.pose.orientation.w = pose_quat_squeeze[0]
        pose_to_drop_squeeze.pose.orientation.x = pose_quat_squeeze[1]
        pose_to_drop_squeeze.pose.orientation.y = pose_quat_squeeze[2]
        pose_to_drop_squeeze.pose.orientation.z = pose_quat_squeeze[3]
        hover_block(pose_to_drop_squeeze, 0.15, speed = 2.5) #hover over squeeze pose
        # return False
        goto_pose_moveit(pose_to_drop_squeeze, speed = 1.5)
        moveit_handler.fa.close_gripper()
        moveit_handler.fa.open_gripper()
        hover_block(pose_to_drop_squeeze, 0.15, speed = 2.5) #hover over squeeze pose

    return True

def composed_pickup(req_pose):
    global fa
    global moveit_handler
    # req_pose obtained by yolo first attempt
    print("Received pose information")
    # Go to pose such that camera is above detected centroid with 15 cm offset
    hover_pose = compute_hover_pose(req_pose)
    hover_done = hover_block(hover_pose, 0.15, speed=3)
    if(hover_done):
        action_goal = GetBlocksGoal(False) #get the center block
        action_client.send_goal(action_goal)
        action_client.wait_for_result()
        action_result = action_client.get_result()
        refined_pose = action_result.pose
        print("Got refined pose")
        # return False
        # hover over the pose
        pickup_done = hover_block(refined_pose, 0.15, speed = 1.5)
        pickup_done = pickup_block(refined_pose)
        pickup_done = hover_block(refined_pose, 0.1, speed=2.5)
        if(pickup_done):
            # moveit_handler.group.attach_object("pickup_block", "panda_hand")
            # moveit_handler.remove_box("pickup_block")
            return True
        else:
            print("Pickup failed")
        return False
    else:
        print("Hover failed")
        return False

def add_obstacles(moveit_handler):
    table_pose = PoseStamped()
    wall_right = PoseStamped()
    wall_front = PoseStamped()
    wall_left = PoseStamped()

    table_pose.header.frame_id = 'panda_link0'
    table_pose.pose.position.x = 0
    table_pose.pose.position.y = 0
    table_pose.pose.position.z = -0.505
    table_pose.pose.orientation.w = 1
    table_pose.pose.orientation.x = 0
    table_pose.pose.orientation.y = 0
    table_pose.pose.orientation.z = 0

    wall_right.header.frame_id = 'panda_link0'
    wall_right.pose.position.x = 0.5
    wall_right.pose.position.y = 0.3
    wall_right.pose.position.z = 0.5
    wall_right.pose.orientation.w = 1
    wall_right.pose.orientation.x = 0
    wall_right.pose.orientation.y = 0
    wall_right.pose.orientation.z = 0

    wall_front.header.frame_id = 'panda_link0'
    wall_front.pose.position.x = 0.7
    wall_front.pose.position.y = 0
    wall_front.pose.position.z = 0.5
    wall_front.pose.orientation.w = 1
    wall_front.pose.orientation.x = 0
    wall_front.pose.orientation.y = 0
    wall_front.pose.orientation.z = 0

    wall_left.header.frame_id = 'panda_link0'
    wall_left.pose.position.x = 0.5
    wall_left.pose.position.y = -0.4
    wall_left.pose.position.z = 0.5
    wall_left.pose.orientation.w = 1
    wall_left.pose.orientation.x = 0
    wall_left.pose.orientation.y = 0
    wall_left.pose.orientation.z = 0

    # moveit_handler.add_box(name='table', pose=table_pose, size=[2, 2, 1])
    # moveit_handler.add_box(name='wall_right', pose=wall_right, size=[1, 0.01, 1])
    # moveit_handler.add_box(name='wall_front', pose=wall_front, size=[0.01, 1, 1])
    # moveit_handler.add_box(name='wall_left', pose=wall_left, size=[1, 0.01, 1])

def scan_for_blocks():
    global action_client
    global moveit_handler

    cartesian_poses_to_scan = []
        
    centroid = np.array([0.6, -0.2, 0.2])
    delta_xy = np.array([[-0.1, -0.1, 0], [-0.1, 0.1, 0], [0,0,0], [0.1, -0.1, 0], [0.1, 0.1, 0]])
    orientation = np.array([0,1,0,0])
    
    # Populate scanning poses
    for i in range(5):
        cartesian_poses_to_scan.append(np.array(centroid + delta_xy[i, :]))
        
    # Loop over poses and call detect blocks
    for i in range(5):
        pose_to_scan = get_posestamped(cartesian_poses_to_scan[i], orientation)
        goto_pose_moveit(pose_to_scan, speed=4.5)
        print("Sending Goal")
        goal = GetBlocksGoal(True)
        action_client.send_goal(goal)
        action_client.wait_for_result()
        result = action_client.get_result()
        print(result)
        if result.pose.header.frame_id == '':
            # Go to next pose
            print("No blocks found in this pose")
            continue
        else:
            return result
    
    return False

# global states
curr_block_id = 0
curr_layer_id = 0
if __name__ == '__main__':
    global moveit_handler
    global action_client
    rospy.init_node('move_it_block_pose_planner')
    print("Node initialized")
    # Setup Moveit Handler
    moveit_handler = moveit_planner()
    # Resetting all obstacles at start
    for obj_id in moveit_handler.scene.get_known_object_names():
        moveit_handler.scene.remove_world_object(obj_id)

    add_obstacles(moveit_handler)
    moveit_handler.fa.reset_joints()
    moveit_handler.fa.open_gripper()
    action_client = actionlib.SimpleActionClient("grasp_pose", GetBlocksAction)
    action_client.wait_for_server()
    while not rospy.is_shutdown():
        moveit_handler.fa.open_gripper()
        # Get Block Pose from Perception service
        des_pose_action_result = scan_for_blocks()
        if(des_pose_action_result == False):
            print("No blocks found lols")
            break
        des_pose = des_pose_action_result.pose # pose stamped
        print("Recieved Goal:\n", des_pose)
        # moveit_handler.add_box('pickup_block', des_pose, size=[0.025, 0.075, 0.015])
        # break

        # pickup block
        pickup_done = composed_pickup(des_pose)
        if(not pickup_done):
            print("Pickup failed")
            break

        # Checking if block has actually been picked up or not
        gripper_width = moveit_handler.fa.get_gripper_width()
        if(gripper_width < 0.07):
            continue

        # drop block
        drop_done = drop_block_moveit(curr_block_id, curr_layer_id)
        if(not drop_done):
            print("Drop failed")
            break

        curr_block_id += 1
        if(curr_block_id == 3):
            curr_block_id = 0
            curr_layer_id += 1
            if(curr_layer_id == 5):
                print("All blocks placed")
                break
