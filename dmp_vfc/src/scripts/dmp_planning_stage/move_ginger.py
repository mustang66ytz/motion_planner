#!/usr/bin/env python
import numpy as np
import rospy
import tf
from robots.ginger import Ginger
import time

# define the ginger pickup home end-effector position
shut_down_signal = False

left_home_trans = [-0.15882225403939296, -0.21258041893974167, 0.08809180992395889]
left_home_quaternion = [3.7469323037635316e-06, 0.24741863338856093, -2.303780439016265e-05, 0.9689086860005726]
right_home_trans = [-0.15882476394746625, 0.21257830005049894, 0.0880896534415361]
right_home_quaternion = [-1.256458299953984e-05, 0.2474087286666777, 3.018587555425712e-05, 0.9689112149265698]

right_home_pos = right_home_quaternion+right_home_trans
left_home_pos = left_home_quaternion+left_home_trans

left_goal_trans = [0.203012626405, -0.10336711683, 0.367727894224]
left_goal_quaternion = [-0.146324317422, 0.813324406607, -0.0763794300819, 0.557905735165]
right_goal_trans = [0.201876243218, 0.105297188055, 0.368928875097]
right_goal_quaternion = [0.150617178566, 0.80436764101, 0.0693176313726, 0.570528047014]

right_pickup_home = [0, 0, 0, 0, 0, 0, 0]
left_pickup_home = [0, 0, 0, 0, 0, 0, 0]

# this function reformats the ginger quaternion order to fit the demo_data collected from yumi
# Do not use this function if demo data is collected from ginger robot
def reformat_ginger_quaternion():
    global left_home_quaternion
    global left_goal_quaternion
    left_home_quaternion = left_home_quaternion[-3:]+left_home_quaternion[0]
    left_goal_quaternion = left_goal_quaternion[-3:]+left_goal_quaternion[0]


def matrix_2_quaternion(matrix):
    ma = np.copy(matrix)
    ma[0, -1] = 0
    ma[1, -1] = 0
    ma[2, -1] = 0

    trans = [None, None, None]
    trans[0] = matrix[0, -1]
    trans[1] = matrix[1, -1]
    trans[2] = matrix[2, -1]

    qua = tf.transformations.quaternion_from_matrix(ma)

    return trans, qua


# this class moves the robot following the result from the dmp service
class MoveGinger(object):

    def __init__(self, gin):
        # initialize a yumi robot
        self.ginger = gin
        # the list of joint states from dmp
        self.traj = []

    def reformat_ginger_quaternion(self):
        for count, waypoint in enumerate(self.traj):
            # chamge the yzwx order to xyzw order
            self.traj[count] = [waypoint[3]]+waypoint[:3]+waypoint[-3:]

    # this function normalizes the generated pose quaternion
    def normalize_quaternion(self):
        for count, pose in enumerate(self.traj):
            norm = np.linalg.norm(pose[:4])
            #norm = pose[0]*pose[0]+pose[1]*pose[1]+pose[2]*pose[2]+pose[3]*pose[3]
            self.traj[count][0] = pose[0]/norm
            self.traj[count][1] = pose[1]/norm
            self.traj[count][2] = pose[2]/norm
            self.traj[count][3] = pose[3]/norm

    # this function is a testing function for the end effector mode control
    def manipulation(self):
        self.ginger.set_main_body_mode(1)
        self.ginger.set_left_hand_mode(1)
        self.ginger.set_right_hand_mode(1)
        self.ginger.set_left_arm_mode(1)
        self.ginger.set_right_arm_mode(1)

        target = [0, 0, 0, 0, 0, 0, 0]
        self.ginger.set_left_arm_joint(target , segmentation = 1)
        self.ginger.set_right_arm_joint(target, segmentation = 1)
        self.ginger.set_left_arm_mode(2)
        self.ginger.set_right_arm_mode(2)

        self.ginger.set_left_arm_velocity(1)
        self.ginger.set_right_arm_velocity(1)

        self.ginger.set_left_arm_mode(4)
        self.ginger.set_right_arm_mode(4)

        print ("The left eef home position is: ", self.ginger.get_left_arm_EEF())
        print ("The right eef home position is: ", self.ginger.get_right_arm_EEF())

        raw_input('Start moving the hand')
        self.ginger.set_left_arm_EEF(left_goal_trans, left_goal_quaternion)
        self.ginger.set_right_arm_EEF(right_goal_trans, right_goal_quaternion)
        raw_input('move the hand back home')
        #self.ginger.set_left_arm_EEF(left_home_trans, left_home_quaternion)
        self.ginger.set_right_arm_EEF(right_home_trans, right_home_quaternion)
        self.ginger.set_left_arm_EEF(left_home_trans, left_home_quaternion)

    def enter_control_mode(self, arm, mode):
        if arm == 'left':
            self.ginger.set_left_arm_mode(mode)
        if arm == 'right':
            self.ginger.set_right_arm_mode(mode)
        else:
            print ("the arm you entered is: ", arm)

    # this function execute the dmp plan to control the end-effector
    def move_e_eff(self, from_home, return_home, arm):
        # get the result file length
        br = tf.TransformBroadcaster()
        with open("resultDMP_ginger_eeff.txt", "r") as result:
            for i, l in enumerate(result):
                pass
            file_length = i + 1
            print("filelength", file_length)
        result.close()

        # get the resulting trajectory
        self.traj = []
        with open("resultDMP_ginger_eeff.txt", "r") as result:
            for line in range(file_length):
                data = eval(result.readline())
                try:
                    #print("data", data)
                    self.traj.append(data)
                except:
                    pass
        result.close()

        # move the ginger simulation following the trajectory from DMP
        self.enter_control_mode(arm, 1)
        if from_home:
            if arm == "right":
                self.ginger.set_right_arm_joint(right_pickup_home)
            elif arm == "left":
                self.ginger.set_left_arm_joint(left_pickup_home)

        # set the control mode for ginger simulation to 4 (end-effector control mode)
        self.enter_control_mode(arm, 4)
        count = 0

        # reformat the ginger quaternion from the yumi quaternion order
        # do not use the following line if the demo data is collected from the ginger robot
        #self.reformat_ginger_quaternion()
        raw_input("wanna execute ? ")

        self.normalize_quaternion()
        time.sleep(2)

        for pose in self.traj:
            try:
                if arm == "right":
                    pass
                    self.ginger.set_right_arm_EEF(pose[-3:], pose[:4])
                elif arm == "left":
                    self.ginger.set_left_arm_EEF(pose[-3:], pose[:4])
                count += 1
                print "exectuting: ", pose[:4]
                time.sleep(0.1)
            except:
                print "infeasible dmp path"
        print "there are "+str(count)+" feasible positions from "+str(len(self.traj))+" positions generated"

        # close the hand
        self.ginger.set_left_hand_mode(1)
        self.ginger.set_left_hand_joint([1, 1, 1, 1, 1])
        rospy.sleep(1)

        if return_home:
            print "The robot is returning home"
            if arm == "right":
                self.ginger.set_right_arm_EEF(right_home_trans, right_home_quaternion)
            if arm == "left":
                #self.ginger.set_left_arm_joint(left_pickup_home)
                self.ginger.set_left_arm_EEF(left_home_trans, left_home_quaternion)


if __name__ == "__main__":
    try:
        # instantiate a ginger control object
        ginger = Ginger()
        # instantiate a move ginger object
        move1 = MoveGinger(ginger)
        # manipulate the ginger in the simulator
        move1.manipulation()
        # move the ginger simulator to execute the dmp plan generated
        #move1.move_e_eff(True, True, "left")
    except:
        pass
