#! /usr/bin/env python
from __future__ import print_function
import actionlib
import action_dmp.msg
from robots.ginger import Ginger
import time
import numpy as np
import rospy
from newik.srv import trajectoryIK


def dmp_action_client(start, goal, file_path):
    # create an action client to the action server named as "dmp_action_server"
    client = actionlib.SimpleActionClient('dmp_action_server', action_dmp.msg.DMPAction)
    # wait for the action server start
    client.wait_for_server()
    goal = action_dmp.msg.DMPGoal(start, goal, file_path)
    client.send_goal(goal)
    client.wait_for_result()
    # print out the result
    return client.get_result()


def enter_control_mode(robot, arm, mode):
    if arm == 'left':
        robot.set_left_arm_mode(mode)
    if arm == 'right':
        robot.set_right_arm_mode(mode)
    else:
        print ("the arm you entered is: ", arm)


# this function normalizes the generated pose quaternion
def normalize_quaternion(traj_2d):
    for count, pose in enumerate(traj_2d):
        norm = np.linalg.norm(pose[:4])
        #norm = pose[0]*pose[0]+pose[1]*pose[1]+pose[2]*pose[2]+pose[3]*pose[3]
        traj_2d[count][0] = pose[0]/norm
        traj_2d[count][1] = pose[1]/norm
        traj_2d[count][2] = pose[2]/norm
        traj_2d[count][3] = pose[3]/norm


def go_home(robot, arm):
    enter_control_mode(robot, arm, 1)
    right_pickup_home = [0, 0, 0, 0, 0, 0, 0]
    left_pickup_home = [0, 0, 0, 0, 0, 0, 0]
    if arm == "right":
        robot.set_right_arm_joint(right_pickup_home)
    elif arm == "left":
        robot.set_left_arm_joint(left_pickup_home, interpolation=None, send_times=5)


def traj_executor_rocky(robot, traj, arm):
    enter_control_mode(robot, 'left', 4)
    raw_input("wanna execute ? ")
    normalize_quaternion(traj)

    for pose in traj:
        try:
            if arm == "right":
                pass
                robot.set_right_arm_EEF(pose[-3:], pose[:4])
            elif arm == "left":
                robot.set_left_arm_EEF(pose[-3:], pose[:4])
            time.sleep(0.1)
        except:
            print ("infeasible dmp path")


def traj_executor_derek(arm_select, traj):
    # execute dmp plan:
    # arm_select: 0 is left; 1 is right
    length = len(traj)
    if arm_select == 0:
        for count, angles in enumerate(traj):
            if count == 0:
                ginger.set_left_arm_mode(1)
            if count < length * 0.7:
                ginger.set_left_arm_joint(angles)
            else:
                ginger.set_left_arm_joint(angles)
            rospy.sleep(0.1)
        print ("successfully executed the current dmp plan on left arm")
    if arm_select == 1:
        for count, angles in enumerate(traj):
            if count == 0:
                ginger.set_right_arm_mode(1)
            if count < length * 0.7:
                ginger.set_right_arm_joint(angles)
            else:
                ginger.set_right_arm_joint(angles)
            rospy.sleep(0.1)
        print ("successfully executed the current dmp plan on right arm")


def reformat_traj(traj_1d):
    traj_2d = []
    temp = []
    for count, item in enumerate(traj_1d):
        if (count + 1) % 7 == 0 and not count == 0:
            temp.append(item)
            traj_2d.append(temp)
            temp = []
        else:
            temp.append(item)
    return traj_2d


if __name__ == "__main__":
    try:
        ginger = Ginger()
        # create a goal to the action server
        initial_pose = [0.08742966306435822, 0.23510704690286763, -0.27011733036133695, 0.9295791428121432, -0.1, -0.2,
                        0.09]
        target_pose = [-0.5571638192069343, 0.575242166261965, -0.21679645766259464, 0.5582689768911218,
                       0.22445238346047316, -0.20288282629715598, 0.3291512751673038]
        mp_file = "./src/action_dmp/src/dmp_learning_primitives/mp1/R_0"

        # uncomment the following to use rocky's IK
        # Rocky's IK starts here
        '''
        result = dmp_action_client(initial_pose, target_pose, mp_file)
        res_1d = result.traj_dmp.data
        res_2d = reformat_traj(res_1d)

        # go home
        go_home(ginger, "left")
        # execute the plan using Rocky's XR-1 IK
        traj_executor_rocky(ginger, res_2d, "left")
        
        '''
        # Rocky's IK end here

        # uncomment the following to use Derek's IK
        # execute the plan using Derek's IK
        # Derek's IK starts here
        arm = "left"
        if "left" == arm:
            arm_select = 0
        elif "right" == arm:
            arm_select = 1
        ginger.set_left_arm_mode(1)
        ginger.set_left_arm_joint([0, 0.2, 0.3, -0.78, 0, 0, 0], interpolation=1)
        rospy.sleep(0.5)
        init_joint_angles = ginger.get_left_arm_joint()
        ginger.set_left_arm_mode(4)
        initial_pose_tuple = ginger.get_left_arm_EEF()
        initial_pose = initial_pose_tuple[1] + initial_pose_tuple[0]
        mwc_left_targets_rot = [-0.16423, 0.75264, 0.010383, 0.63754]
        mwc_left_targets_trans = [0.16535, -0.28777, 0.27845]
        target_pose = mwc_left_targets_rot + mwc_left_targets_trans

        result = dmp_action_client(initial_pose, target_pose, mp_file)
        res_1d = result.traj_dmp.data
        # call the newik ROS service
        try:
            rospy.wait_for_service('provide_ik_service')
            ik_server = rospy.ServiceProxy('provide_ik_service', trajectoryIK)
            newik_mode = "1"
            num_waypoint = len(res_1d) / 7
            resp = ik_server(arm_select, int(newik_mode), num_waypoint, init_joint_angles, res_1d)
            failRange = resp.failNum
            resAngles = resp.jointAngles
            resScore = resp.success
            res_2d = reformat_traj(resAngles)
            if resScore == 1.0:
                raw_input("Wanna execute?")
                traj_executor_derek(arm_select, res_2d)
            else:
                print ("IK is not able to find a solution")
        except rospy.ServiceException, e:
            print ("Servie call failed: %s" % e)
        # Derek's IK ends here
        
    except:
        print("program interrupted before completion")

