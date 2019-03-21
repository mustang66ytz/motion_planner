#!/usr/bin/env python
import roslib
roslib.load_manifest('dmp')
import rospy
import numpy as np
import os
from dmp.srv import *
from dmp.msg import *
import time


# Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, D_gain, num_bases):
	# initialize a dmp trajectory message
	demotraj = DMPTraj()

	for i in range(len(traj)):
		# initialize a dmp point
		pt = DMPPoint()
		# assign the points in the trajectory to the DMP point
		pt.positions = traj[i]
		# append the new point to the trajectory
		demotraj.points.append(pt)
		demotraj.times.append(dt * i)

	k_gains = [K_gain] * dims
	d_gains = [D_gain] * dims

	print "Starting LfD..."
	rospy.wait_for_service('learn_dmp_from_demo')
	try:
		lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
		resp = lfd(demotraj, k_gains, d_gains, num_bases)
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e
	print "LfD done"
	return resp


# Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):

	try:
		sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
		sad(dmp_list)
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e


# Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
					seg_length, tau, dt, integrate_iter):
	print "Starting DMP planning..."
	rospy.wait_for_service('get_dmp_plan')
	try:
		gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
		resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
				   seg_length, tau, dt, integrate_iter)
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e
	print "DMP planning done"
	return resp


def recordPlan(points, mode):
	recordedData = None
	dim = 7

	if mode == "joints":
		recordedData = open("resultDMPJoints.txt", 'w')

	if mode == "ee":
		recordedData = open("resultDMP_ginger_eeff.txt", 'w+')

	for item in points:
		joints = []
		for i in range(dim):
			joints.append(item.positions[i])
		recordedData.write(str(joints))
		recordedData.write("\n")
	recordedData.close()


# run the dmp if trained on robot's joint states
def dmp_joints():
	try:
		# Create a DMP from a 7-D joint space trajectory
		# customize some hyper-parameters
		dims = 7
		dt = 0.5  # incremental time
		K = 100  # proportional gain
		D = 2.0 * np.sqrt(K)  # derivative gain
		num_bases = 20  # number of gaussian primitives
		inputJoints = []  # store the input demo data

		# read training data from file

		with open("listOfJoints1.txt", "r") as dataRecorded:
			for line in range(10):
				temp = []
				data = eval(dataRecorded.readline())
				for item in data:
					temp.append(item)
				print(temp)
				inputJoints.append(temp)
		dataRecorded.close()

		resp = makeLFDRequest(dims, inputJoints, dt, K, D, num_bases)
		makeSetActiveRequest(resp.dmp_list)

		goal_thresh = [2, 2, 2, 2, 2, 2, 2]
		seg_length = -1  # Plan until convergence to goal
		tau = 2 * resp.tau  # Desired plan should take 8 times as long as demo
		dt = 0.2
		integrate_iter = 5  # dt is rather large, so this is > 1

		# customize the start and goal configurations
		# start_config = [-70, -114.45, 37.59, 130, 102.66, -8.73, 42]
		start_config = [-110.93, -118.7, 54.16, 144.39, 125.69, -3.43, 42.01]
		# end_config = [-70, -114.45, 37.57, 180, 29.59, -8.92, 42]
		end_config = [-128.49, -120.08, 49.5, 183.56, 122.51, -7.56, 42.01]
		joints_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		t_0 = 0

		plan = makePlanRequest(start_config, joints_dot_0, t_0, end_config, goal_thresh,
							   seg_length, tau, dt, integrate_iter)
		recordPlan(plan.plan.points, "joints")

	except rospy.ROSInterruptException:
		pass


# run the dmp if trained on robot's end-effector positions:
def dmp_ee(filename, start, target):
	try:
		# Create a DMP from a 7-D joint space trajectory
		# customize some hyper-parameters
		dims = 7
		dt = 1  # incremental time
		K = 50  # proportional gain
		D = 2.0 * np.sqrt(K)  # derivative gain
		num_bases = 10  # number of gaussian primitives
		inputStates = []  # store the input demo data

		# read training data from file
		file_length = 0
		with open(filename, "r") as dataRecorded:
			# with open("mp1/R_1", "r") as dataRecorded:
			for i, l in enumerate(dataRecorded):
				pass
			file_length = i + 1
		dataRecorded.close()

		with open(filename, "r") as dataRecorded:
			for line in range(file_length):
				temp = []
				data = eval(dataRecorded.readline())
				for item in data:
					temp.append(item)
				inputStates.append(temp)
		dataRecorded.close()

		resp = makeLFDRequest(dims, inputStates, dt, K, D, num_bases)
		makeSetActiveRequest(resp.dmp_list)

		goal_thresh = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
		seg_length = -1  # Plan until convergence to goal
		tau = resp.tau  # Desired plan should take the same time as that of the demo
		dt = 1
		integrate_iter = 5  # dt is rather large, so this is > 1

		start_config = start
		end_config = target

		joints_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		t_0 = 0

		plan = makePlanRequest(start_config, joints_dot_0, t_0, end_config, goal_thresh,
							   seg_length, tau, dt, integrate_iter)
		recordPlan(plan.plan.points, "ee")

	except rospy.ROSInterruptException:
		pass


# this is the main entry point to handle parameter collection, waypoints generation, and dmp execution
def move_primitives(ik_selection, targets, primitive_folder, arm, ginger, newik_mode):
	try:
		# cd to the motion_primitives folder:
		orig_dir = os.getcwd()
		os.chdir("../dmp_learning_primitives")

		primitive_filenames = os.listdir(primitive_folder)  # list all the files in the motion primitive folder
		primitive_filenames.sort()
		del primitive_filenames[-1]  # remove the last one, which is the dmp_params
		print primitive_filenames

		if ik_selection == "oldik":
			import move_ginger as mvgg
			move1 = mvgg.MoveGinger(ginger)
			print "moving the ginger"
			# calibrate the arm and gripper first
			move1.ginger.set_left_arm_mode(1)
			move1.ginger.set_right_arm_mode(1)
			# generate the from home list and the return home list
			from_home = []
			return_home = []
			for idx in range(len(primitive_folder) - 1):
				from_home.append(False)
				return_home.append(False)
			from_home[0] = True
			return_home[-1] = True

		# check is the length of primitives one less than the length of the target way points,
		# execute the dmp if it is true, print error message if it is false
		if len(primitive_filenames) == len(targets)-1:

			for counter, primitive in enumerate(primitive_filenames):
				print "Generating the motion primitive-------------------------"
				print "primitive:", primitive
				# run the dmp algorithm to generate a plan
				dmp_ee(primitive_folder+"/"+primitive, targets[counter], targets[counter+1])
				print "from:", targets[counter]
				print "to:", targets[counter+1]
				print "start the first primitive ----------------------------"
				# execute the generated dmp plan on the robot
				# old ik, not recommended
				if ik_selection == "oldik":
					move1.move_e_eff(from_home[counter], return_home[counter], arm)
				# new ik, recommended
				elif ik_selection == "newik":
					import newikClient as newmvgg
					move2 = newmvgg.MoveGinger(ginger)
					# set the current joint angle as the initial angle for dmp planning
					move2.get_init_angles(counter)
					initAngles = move2.init_angles
					inputPoints = move2.read_result_new()
					# choose the mode for planning
					# 1: trajectory ik
					# 2: linear ik
					# 3: point ik
					num_waypoint = 0
					if "1" == newik_mode:
						num_waypoint = len(inputPoints) / 7
					elif "2" == newik_mode:
						pass
					elif "3" == newik_mode:
						num_waypoint = 1
					# call the new ik client to execute the plan
					result = move2.new_ik_client(arm, newik_mode, num_waypoint, initAngles, inputPoints)
					# evaluate the execution result:
					if not result:
						break
				time.sleep(1)
			if result:
				return result, None
			else:
				return False, counter
		else:
			print "The number of target way points does not match the number of motion primitives"
		# exit back to the original directory
		os.chdir(orig_dir)
	except:
		pass

