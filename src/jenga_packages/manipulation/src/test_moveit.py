#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

sys.path.append("/home/ros_ws/src/jenga_packages/frankapy")
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import min_jerk

class moveit_planner():
    def __init__(self) -> None: #None means no return value
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

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
    

    def reset_joints(self):
        # joint_goal = self.group.get_current_joint_values()
        # joint_goal[0] = 0.0
        # joint_goal[1] = -0.7853981633974483
        # joint_goal[2] = 0
        # joint_goal[3] = -2.356194490192345
        # joint_goal[4] = 0
        # joint_goal[5] = 1.5707963267948966
        # joint_goal[6] = 0.7853981633974483
        # self.goto_joint(joint_goal)
        self.fa.reset_joints()

    def goto_joint(self, joint_goal):
        # for simulation
        # self.group.go(joint_goal, wait=True)
        # self.group.stop()

        # for real robot use frankapy
        self.fa.goto_joints(joint_goal, duration=5, dynamic=True, buffer_time=10)
    
    def get_plan_given_pose(self, pose_goal: geometry_msgs.msg.Pose):
        output = self.group.plan(pose_goal)
        plan = output[1]
        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values
    
    def get_plan_given_joint(self, joint_goal_list):
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

    def execute_plan(self, joints_traj):
        '''
        joints_traj shape: (N x 7)
        '''
        # self.group.execute(joints_traj, wait=True)
        min_jerk_traj = []
        ts = np.arange(0, 0.5, 0.02)
        for i in range(1, joints_traj.shape[0]):
            min_jerk_traj_i = [min_jerk(joints_traj[i-1], joints_traj[i], t, 0.5) for t in ts]
            min_jerk_traj.extend(min_jerk_traj_i)
        min_jerk_traj = np.array(min_jerk_traj)
        print('min_jerk_traj shape: ', min_jerk_traj)

        # for franka robot
        print('Executing joints trajectory of shape: ', min_jerk_traj.shape)
        rate = rospy.Rate(50)
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(min_jerk_traj[1], duration=5, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(2, min_jerk_traj.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=min_jerk_traj[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            
            print('Publishing: ID {}'.format(traj_gen_proto_msg.id))
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
        # import matplotlib.pyplot as plt
        # plt.plot(min_jerk_traj[:, 0])
        # plt.plot(min_jerk_traj[:, 1])
        # plt.plot(min_jerk_traj[:, 2])
        # plt.show()
    
    def run_guide_mode(self, time):
        self.fa.run_guide_mode(time)

if __name__ == "__main__":
    franka_moveit = moveit_planner()
    franka_moveit.reset_joints()
    # ros spin loop
    # rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    # rospy.spin()
    
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.x = -0.0237213
    # pose_goal.orientation.y = 0.99928303
    # pose_goal.orientation.z = -0.02684862
    # pose_goal.orientation.w = 0.01204441
    # pose_goal.position.x = 0.41160904
    # pose_goal.position.y = -0.11861613
    # pose_goal.position.z = 0.24213254
    
    # plan_pose = franka_moveit.get_plan_given_pose(pose_goal)
    # print(plan_pose.shape, "pose plan shape")

    # get joint state plan
    joint_goal_list = [6.14813255e-02 ,4.11382927e-01, 6.80023936e-02,-2.09547337e+00,-2.06094866e-03,2.56799173e+00,  9.20088362e-01]
    plan_joint_vals, plan_joint = franka_moveit.get_plan_given_joint(joint_goal_list)

    print(plan_joint_vals.shape, "plan_joint_vals shape")
    franka_moveit.execute_plan(plan_joint_vals)

    # franka_moveit.reset_joints()
# error
#original
# 6.14813255e-02  4.11382927e-01  6.80023936e-02 -2.09547337e+00
#  -2.06094866e-03  2.56799173e+00  9.20088362e-01

#moved to
# 6.06152662e-02  4.07360911e-01  6.74233853e-02 -2.10341030e+00
#  -1.18097945e-03  2.54677025e+00  9.06812495e-01