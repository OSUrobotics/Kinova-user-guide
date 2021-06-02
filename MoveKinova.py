#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 4/2021


import rospy
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import time



class MoveKinova():
	"""Use moveit to move the kinova arm. This class encapsulates most moveit methods
	that are available."""
	def __init__(self):
		# Initialize moveit commander and ros node for moveit
		moveit_commander.roscpp_initialize(sys.argv)

		# Initializing node
		# rospy.init_node("moveit_kinova", anonymous=True)

		# Define robot using RobotCommander.
		self.robot = moveit_commander.RobotCommander()

		# Setting the world
		self.scene = moveit_commander.PlanningSceneInterface()

		self.move_group = moveit_commander.MoveGroupCommander("arm")
		self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

		# Set robot precision
		rospy.set_param("/move_group/trajectory_execution/allowed_start_tolerance", 0.0)

		rospy.wait_for_service("/apply_planning_scene", 10.0)
		rospy.wait_for_service("/get_planning_scene", 10.0)


		self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
		self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
		# rospy.sleep(2)

		
		# To see the trajectory
		self.disp = moveit_msgs.msg.DisplayTrajectory()

		self.disp.trajectory_start = self.robot.get_current_state()
		
	
	def set_planner_type(self, planner_name):
		if planner_name == "RRT":
			self.move_group.set_planner_id("RRTConnectkConfigDefault")
		if planner_name == "RRT*":
			self.move_group.set_planner_id("RRTstarkConfigDefault")
		if planner_name == "PRM*":
			self.move_group.set_planner_id("PRMstarkConfigDefault")


	def go_to_joint_state(self, joint_state):
		joint_goal = JointState()
		joint_goal.position = joint_state
		self.move_group.set_joint_value_target(joint_goal.position)

		self.plan = self.move_group.plan()
		# self.move_group.go(wait=True)
		self.move_group.execute(self.plan, wait=True)

	
	def go_to_pose(self, ee_pose, wait_for_execution):
		"""Goes to the pose with both euler and quaternion angles. If you want the goal pose
		to preemptable, set wait for execution to False. Else set it True for complete execution"""
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = ee_pose[0]
		pose_goal.position.y = ee_pose[1]
		pose_goal.position.z = ee_pose[2]
		
		if len(ee_pose) == 6:
			quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]), math.radians(ee_pose[5]))
			pose_goal.orientation.x = quat[0]
			pose_goal.orientation.y = quat[1]
			pose_goal.orientation.z = quat[2]
			pose_goal.orientation.w = quat[3]

		else:
			pose_goal.orientation.x = ee_pose[3]
			pose_goal.orientation.y = ee_pose[4]
			pose_goal.orientation.z = ee_pose[5]
			pose_goal.orientation.w = ee_pose[6]	

		self.move_group.set_pose_target(pose_goal)
		self.move_group.set_planning_time(20)
		

		# rospy.sleep(2)
		self.move_group.go(wait= wait_for_execution)

		# self.move_group.allow_replanning(1)
		# self.move_group.stop()

		# self.move_group.clear_pose_targets()
		# rospy.sleep(2)

	def move_fingers(self, cmd):
		if cmd == "close":
			self.move_gripper.set_named_target("Close")
		elif cmd == "Open":
			self.move_gripper.set_named_target("Open")
		else: 
			self.move_gripper.set_joint_value_target(cmd)
		self.move_gripper.go(wait=True)
		

	def display_trajectory(self):
		self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.disp.trajectory.append(self.plan)
		print(self.disp.trajectory)
		self.disp_pub.publish(self.disp)

	def stop_execution(self):
		self.move_group.stop()

	def remove_previous_goal_target(self):
		self.move_group.clear_pose_targets()


	def axis_offset_target_pose(self, offset_list):
		"""Shift each value (x, y, z, roll, pitch, yaw) by the a certain amount.
		This gets the current pose and offsets the current value by delta
		:param: offset_list - (detla_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw)"""

		delta_x = offset_list()
		

	def replan_if_failed(self, count):
		"""Handles replanning if it fails the first time."""
		self.move_group.allow_replanning(count)


	def get_current_pose(self):
		return self.move_group.get_current_pose()


	def execute_plan(self, plan, wait_for_execution):
		self.move_group.execute(plan, wait=wait_for_execution)




if __name__ == '__main__':
	rospy.init_node("moveit_kinova", anonymous=True)

	kinova = MoveKinova()

	kinova.set_planner_type("RRT")

	pose = [-0.1, -0.63, 0.2, 0, 180, 0]

	rospy.loginfo('Going to point')

	kinova.go_to_pose(pose, True)
	

	# # Time delays to see the preemptions
	time.sleep(2.0)
	kinova.stop_execution()
	rospy.loginfo("Stopping")
	time.sleep(2.0)

	kinova.remove_previous_goal_target()

	pose = [0.1, -0.63, 0.2, 0, 180, 0] 

	rospy.loginfo("Preempting")
	kinova.go_to_pose(pose, False)
	time.sleep(2.0)
	kinova.stop_execution()

	current_pose = kinova.get_current_pose()

	position = (current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)

	print(current_pose)
	print(position)

