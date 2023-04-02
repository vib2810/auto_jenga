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
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -0.7853981633974483
        joint_goal[2] = 0
        joint_goal[3] = -2.356194490192345
        joint_goal[4] = 0
        joint_goal[5] = 1.5707963267948966
        joint_goal[6] = 0.7853981633974483
        self.goto_joint(joint_goal)

    def goto_joint(self, joint_goal):
        # for simulation
        self.group.go(joint_goal, wait=True)
        self.group.stop()

        # for real robot use frankapy
    
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
        
        # for franka robot
        rospy.loginfo('Executing joints trajectory of shape: ', joints_traj.shape())
        rate = rospy.Rate(50)
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(joints_traj[1], duration=5, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(2, joints_traj.shape[1]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=joints_traj[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            
            rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
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

if __name__ == "__main__":
    franka_moveit = moveit_planner()
    franka_moveit.reset_joints()

    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    
    plan_pose = franka_moveit.get_plan_given_pose(pose_goal)
    print(plan_pose.shape, "pose plan shape")

    # get joint state plan
    joint_goal_list = [0.0, -0.7853981633974483, 0, -1.2, 0, 1.5707963267948966, 0.7853981633974483]
    plan_joint_vals, plan_joint = franka_moveit.get_plan_given_joint(joint_goal_list)

    print(plan_joint_vals.shape, "plan_joint_vals shape")
    franka_moveit.execute_plan(plan_joint)

    franka_moveit.reset_joints()