import numpy as np
import sys
from robots.ginger import Ginger
import threading

# home position joint angle
left_home = [0.1, 0, 0, 0, 0, 0, 0]
right_home = [0.1, 0, 0, 0, 0, 0, 0]

# home position end effector
left_home_trans = [-0.11052067528539275, 0.2538157436243712, -0.37804247032096616]
left_home_quaternion = [0.6970655586405908, 0.12306331875940428, 0.6956035991335582, -0.12284413948871706]
right_home_trans = [-0.11037999736179446, -0.2538949353857735, -0.3780378356874762]
right_home_quaternion = [0.6968756615037409, -0.12234712293326104, 0.6958088677997202, 0.12347281724520806]

left_goal_trans = [-0.15085968482494272, 0.2384207567179582, -0.37600253162282715]
left_goal_quaternion = [0.7263637196083014, 0.14147523355528457, 0.6601944246486715, -0.12854512882113237]
right_goal_trans = [-0.15061843288558258, -0.23843220555182026, -0.37603121233531844]
right_goal_quaternion = [0.725840478097858, -0.14045978009414561, 0.660801806116033, 0.12948994905756708]
shut_down_signal = False

def set_speed(gin, speed):
	while not shut_down_signal:
		gin.set_left_arm_velocity(speed)
		gin.set_right_arm_velocity(speed)
		gin.rate.sleep()

def left_go_home(gin):
	while not shut_down_signal:
		ginger.set_left_arm_mode(4)
		gin.set_left_arm_EEF(left_goal_trans, left_goal_quaternion)
		#gin.set_left_arm_joint(left_home)
		gin.rate.sleep()

def right_go_home(gin):
	while not shut_down_signal:
		ginger.set_right_arm_mode(4)
		gin.set_right_arm_EEF(right_goal_trans, right_goal_quaternion)
		#gin.set_right_arm_joint(right_home)
		gin.rate.sleep()

def rigid_to_quaternion_position(R):
	return R.quaternion.tolist() + np.reshape(R.position, 3).tolist()

def display_info(gin):
	while not shut_down_signal:
		print "posL: ", gin.get_left_arm_EEF()
		print "posR: ", gin.get_right_arm_EEF()
		gin.rate.sleep()

if __name__ == "__main__":
	ginger = Ginger()

	# set the speed
	joint_angle_speed = [0.3, 1, 1, 1, 1, 1, 1]
	#mode = 1
	#ginger.set_left_arm_mode(mode)
	#ginger.set_right_arm_mode(mode)

	speed_control_thread = threading.Thread(target=set_speed, args=(ginger, joint_angle_speed,), name='set speed')
	left_position_control_thread = threading.Thread(target=left_go_home, args=(ginger,), name='set left arm position')
	right_position_control_thread = threading.Thread(target=right_go_home, args=(ginger,), name='set right arm position')
	monitor_thread = threading.Thread(target=display_info, args=(ginger,), name='monitor the current position')

	speed_control_thread.start()
	left_position_control_thread.start()
	right_position_control_thread.start()
	monitor_thread.start()

	speed_control_thread.join()
	left_position_control_thread.join()
	right_position_control_thread.join()
	monitor_thread.join()




