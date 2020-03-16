#!/usr/bin/env python

import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension, MultiArrayLayout 
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState

# stepsPerRevolution= []
prev_angle = []
count = 0
joint_status = 0

# def callback(cmd_arm):
# 	'''
# 	Define a function that translates absolute angles to relative positions
# 	'''
# 	if (count==0):
# 		for i in range(0,len(cmd_arm.position)):
# 			prev_angle[i] = cmd_arm.position[i]
# 			# prev_angle[0] = cmd_arm.position[0]
# 			# prev_angle[1] = cmd_arm.position[1]
# 			# prev_angle[2] = cmd_arm.position[2]
# 			# prev_angle[3] = cmd_arm.position[3]
# 			# prev_angle[4] = cmd_arm.position[4]
# 			# prev_angle[5] = cmd_arm.position[5]
# 			init_angle[i] = cmd_arm.position[i]
# 			# init_angle[0] = cmd_arm.position[0]
# 			# init_angle[1] = cmd_arm.position[1]
# 			# init_angle[2] = cmd_arm.position[2]
# 			# init_angle[3] = cmd_arm.position[3]
# 			# init_angle[4] = cmd_arm.position[4]
# 			# init_angle[5] = cmd_arm.position[5]
# 		count = 1

# 	for i in range(0,len(prev_angle)):
# 		arm_steps.position[i] = (cmd_arm.position[i]-prev_angle[i])
# 		prev_angle[i] = cmd_arm.position[i]

# 	joint_status = 1


class MoveIt_Arduino:
	def __init__(self):
		self.prev_angle = []
		print(self.prev_angle)
		self.init_angle = []
		self.count = 0
		# self.joint_status = 0

		self.arduino_publisher = rospy.Publisher("arduino_cmd",Float64MultiArray, queue_size=10)

		self.moveit_subs = rospy.Subscriber("execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.callback)
		self.arm_steps = JointState()
		print("Initialized Node")

	def callback(self,cmd_arm):
		'''
		Define a function that translates absolute angles to relative positions
		'''
		
		# print(cmd_arm)
		# waypoints = Float64MultiArray()
		wp = []
		# i = 1
		# waypoints = cmd_arm.goal.trajectory.joint_trajectory.points
		for point in cmd_arm.goal.trajectory.joint_trajectory.points:
			# print("Waypoint number {}: {}".format(i,point.positions))
			pose = []
			for pos in point.positions:
				wp.append(pos)
			# wp.append(pose)
			# i += 1
		# print(type(point.positions))	
		# print(type(wp))
		# print(waypoints)
		# waypoints.layout.dim = [len(cmd_arm.goal.trajectory.joint_trajectory.points),6]
		
		# dim = MultiArrayDimension('label','size','stride')
		# dim.label = 'waypoints'
		# dim.size = len(cmd_arm.goal.trajectory.joint_trajectory.points)
		# dim.stride = 1
		# waypoints.layout.dim.append(dim)

		# waypoints.layout.dim[0].label = 'waypoints'
		# waypoints.layout.dim[0].label = 'waypoints'
		# waypoints.layout.dim = {'label': 'waypoints', 'size': len(cmd_arm.goal.trajectory.joint_trajectory.points),'stride': 1}
		
		# waypoints.data = wp
		dat = np.zeros((3,2))
		dat1 = [1.0,2.0,3.0]
		dat2 = tuple(dat)
		waypoints = Float64MultiArray(data=wp, layout=MultiArrayLayout(data_offset=len(wp)))
		print(waypoints)
		
		# print("Running Callback")
		# if (self.count==0):
		# 	for i in range(0,len(cmd_arm.position)):
		# 		print("Retrieving joint {}".format(i))
		# 		self.prev_angle.append(cmd_arm.position[i])
		# 		# prev_angle[0] = cmd_arm.position[0]
		# 		# prev_angle[1] = cmd_arm.position[1]
		# 		# prev_angle[2] = cmd_arm.position[2]
		# 		# prev_angle[3] = cmd_arm.position[3]
		# 		# prev_angle[4] = cmd_arm.position[4]
		# 		# prev_angle[5] = cmd_arm.position[5]
		# 		self.init_angle.append(cmd_arm.position[i])
		# 		# init_angle[0] = cmd_arm.position[0]
		# 		# init_angle[1] = cmd_arm.position[1]
		# 		# init_angle[2] = cmd_arm.position[2]
		# 		# init_angle[3] = cmd_arm.position[3]
		# 		# init_angle[4] = cmd_arm.position[4]
		# 		# init_angle[5] = cmd_arm.position[5]
		# 	self.count = 1

		# new_cmd = []
		# for i in range(0,len(cmd_arm.position)):
		# 	new_cmd.append(cmd_arm.position[i]-self.prev_angle[i])
		# 	self.prev_angle[i] = cmd_arm.position[i]
		# self.arm_steps.position = new_cmd
		# # print(self.arm_steps)
		# # self.joint_status = 1
		self.arduino_publisher.publish(waypoints)


# def main():
# 	rospy.init_node('moveit_arduino_middleware', anonymous=True)

# 	arduino_publisher = rospy.Publisher('/arduino_cmd',JointState, queue_size=10)

# 	moveit_subs = rospy.Subscriber('/joint_states', JointState, callback)
# 	rate = rospy.Rate(10) # 10hz
# 	arm_steps = JointState
# 	while not rospy.is_shutdown():
# 		if joint_status == 1:
# 			joint_status = 0
# 			arduino_publisher.publish(arm_steps)

# 		rospy.spin_once()
# 		rate.sleep()

if __name__ == '__main__':
	fail = 0
	# try:
	print("Init Node")
	rospy.init_node('moveit_arduino_middleware', anonymous=False)
	print("Create Instance of class")
	ard_cont = MoveIt_Arduino()

	# except rospy.ROSInterruptException:
	# 	print("Failed to create node and class.")
	# 	fail = 1
	print("Starting rospy loop")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rospy.spin()
		rate.sleep()

		