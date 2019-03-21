#!/usr/bin/env python
import numpy as np
import rospy
from robots.ginger import Ginger
from newik.srv import trajectoryIK
from dmp_vfc.msg import *
import dmp_utils

np.set_printoptions(precision=4)


class MoveGinger(object):

    def __init__(self, gin):
        self.init_angles = []
        self.init_pos = []
        # initialize a yumi robot
        self.ginger = gin
        # the list of poses
        self.traj_pts = []
        # the list of joint states from dmp
        self.traj = []

    def reformat_result(self, result):
        self.traj = []
        temp = []
        for count, item in enumerate(result):
            if (count + 1) % 7 == 0 and not count == 0:
                temp.append(item)
                self.traj.append(temp)
                temp = []
            else:
                temp.append(item)

    def execute_plan(self, arm_select):
        # execute dmp plan:
        # arm_select: 0 is left; 1 is right
        length = len(self.traj)
        if arm_select == 0:
            for count, angles in enumerate(self.traj):
                if count == 0:
                    self.ginger.set_left_arm_mode(1)
                if count < length * 0.7:
                    self.ginger.set_left_arm_joint(angles)
                else:
                    self.ginger.set_left_arm_joint(angles)
                rospy.sleep(0.1)
            print "successfully executed the current dmp plan on left arm"

        if arm_select == 1:
            for count, angles in enumerate(self.traj):
                if count == 0:
                    self.ginger.set_right_arm_mode(1)
                if count < length * 0.7:
                    self.ginger.set_right_arm_joint(angles)
                else:
                    self.ginger.set_right_arm_joint(angles)
                rospy.sleep(0.1)
            print "successfully executed the current dmp plan on right arm"

        # The following section can be replaced by vfc module for precise manipulation:
        '''
        # change to linear mode here:
        ik_server_linear = rospy.ServiceProxy('provide_ik_service', trajectoryIK)
        if arm_select == 0:
            init_angle = self.traj[-1]
            print "The init angles is: ", init_angle
        elif arm_select == 1:
            init_angle = self.traj[-1]

        # this is the dmp final pose tf listener
        lr = tf.TransformListener()
        lr.waitForTransform('/Back_Y', '/LinearTarget', rospy.Time(), rospy.Duration(0.1))
        (final_trans, final_qua) = lr.lookupTransform('/Back_Y', '/LinearTarget', rospy.Time(0))
        final_pose = final_qua + final_trans
        resp_linear = ik_server_linear(arm_select, 2, 5, init_angle, final_pose)
        failRange = resp_linear.failNum
        resAngles = resp_linear.jointAngles
        resScore = resp_linear.success

        print "linear planning success score: ", resScore
        print "The failed range is: ", failRange

        # execute the linear motion:
        self.reformat_result(resAngles)
        raw_input("Wanna execute?")
        if resScore == 1.0:
            for count, angles in enumerate(self.traj):
                if count == 0:
                    if arm_select == 0:
                        self.ginger.set_left_arm_mode(1)
                    elif arm_select == 1:
                        self.ginger.set_right_arm_mode(1)
                    rospy.sleep(0.1)
                if arm_select == 0:
                    self.ginger.set_left_arm_joint(angles)
                elif arm_select == 1:
                    self.ginger.set_right_arm_joint(angles)
                rospy.sleep(0.1)
            #print "the current joint angle is: ", self.ginger.get_left_arm_joint()
            #self.ginger.set_left_arm_mode(4)
            #rospy.sleep(0.1)
            #print "the current eef position is: ", self.ginger.get_left_arm_EEF()
        else:
            print "Fail, cannot execute"

        # close the hand
        self.ginger.set_left_hand_mode(1)
        self.ginger.set_left_hand_joint([1.2]*5)

        '''

    # this function smoothen the dmp trajectory with a linear plan
    def modify_dmp_traj(self, traj, iteration):
        # modified_traj: the final modified trajectory
        # linear_traj: the linear trajectory formed by the projection from the original trajectory
        # original_traj: the original dmp trajectory
        # weight: the weight for linear projection point, and 1-weight for original dmp point
        modified_traj = []
        linear_traj = []
        original_traj = []
        temp = []
        weight = iteration * 0.05

        for count, item in enumerate(traj):
            if (count + 1) % 7 == 0 and not count == 0:
                temp.append(item)
                original_traj.append(temp)
                temp = []
            else:
                temp.append(item)

        # processing the translation first:

        # targetPt: the goal translation
        # startPt: the starting point translation
        # ab: vector from startPt to targetPt
        # ap: vector from startPt to waypoint on original dmp trajectory
        # intersectPt: the projected point from dmp point to the staight line
        targetPt_trans = np.asarray(original_traj[-1][-3:])
        startPt_trans = np.asarray(original_traj[0][-3:])
        ab = np.subtract(targetPt_trans, startPt_trans)

        for projectPt in original_traj:
            projectPt_trans = np.asarray(projectPt[-3:])
            ap = np.subtract(projectPt_trans, startPt_trans)
            intersectPt_trans = startPt_trans + np.dot(ap, ab) / np.dot(ab, ab) * ab
            linear_traj.append(intersectPt_trans)

        # Processing the orientation:

        targetPt_qua = np.asarray(original_traj[-1][:4])
        startPt_qua = np.asarray(original_traj[0][:4])
        delta_qua = np.subtract(targetPt_qua, startPt_qua)

        # calculate the length proportion for every intersectPt
        total_delta_x = linear_traj[-1][0] - linear_traj[0][0]
        length_portion = []
        for intersectPt_trans in linear_traj:
            portion = (intersectPt_trans[0] - linear_traj[0][0]) / total_delta_x
            length_portion.append(portion)

        # interpolate the quaternion for every point
        linear_traj_full = []
        for intersectPt_trans, portion in zip(linear_traj, length_portion):
            intersectPt_qua = startPt_qua + delta_qua * portion
            intersectPt = np.hstack((intersectPt_qua, intersectPt_trans))
            linear_traj_full.append(intersectPt)

        # obtain the modified trajectory by averaging the projected trajectory and the original dmp traj
        for intersectPt, projectPt in zip(linear_traj_full, original_traj):
            projectPt = np.asarray(projectPt)
            averagePt = weight * intersectPt + (1 - weight) * projectPt
            modified_traj.append(averagePt)

        # convert the np array to a 1D list
        final_traj = []
        for row in modified_traj:
            for item in row:
                final_traj.append(item)
        print "The processed trajectory is: ", final_traj
        return final_traj

    def new_ik_client(self, arm, newik_mode, num_waypoints, initialJoints, wayPoints):
        self.init_angles = initialJoints
        rospy.wait_for_service('provide_ik_service')
        # call the ik service and print the result generated
        try:
            if "left" == arm:
                arm_select = 0
            elif "right" == arm:
                arm_select = 1

            ik_server = rospy.ServiceProxy('provide_ik_service', trajectoryIK)

            # handle the failed planning case until the resScore equals one:
            # wayPoints = self.modify_dmp_traj(wayPoints)
            haha = dmp_utils.convert1dto2d(wayPoints)
            # visualize the planning success
            marker_pose_pub = rospy.Publisher('trajectory_pose', Waypoint, queue_size=10)
            fail_range_pub = rospy.Publisher('fail_range', Waypoint, queue_size=10)
            # get response from the server
            print "num_waypoints:", num_waypoints
            resp = ik_server(arm_select, int(newik_mode), num_waypoints, self.init_angles, wayPoints)
            failRange = resp.failNum
            resAngles = resp.jointAngles
            resScore = resp.success

            # automatic parameter tunning:
            '''
            iteration = 0
            while not resScore == 1 and iteration<20:
                wayPoints = self.modify_dmp_traj(wayPoints, iteration)
                iteration = iteration + 1
                resp = ik_server(arm_select, int(newik_mode), len(wayPoints)/7, self.init_angles, wayPoints)
                resScore = resp.success
                resAngles = resp.jointAngles
                print "Tunning the dmp plan with linear plan: ", iteration
                print "Current success rate is: ", resScore
            '''

            # define the msg to be published
            marker_pose_back_msg = Waypoint()
            marker_pose_back_msg.target = wayPoints
            fail_range_num_msg = Waypoint()
            fail_range_num_msg.target = failRange
            for i in range(10):
                marker_pose_pub.publish(marker_pose_back_msg)
                rospy.sleep(0.1)
                fail_range_pub.publish(fail_range_num_msg)
                rospy.sleep(0.1)

            # reformatting the resulting angles here
            self.reformat_result(resAngles)

            # print "the resulting angles are:"
            # for count, angles in enumerate(self.traj):
            #    print "the number: ", count
            #    print angles
            print "the result score is: ", resScore
            print "the failed range is: ", failRange

            # execute the plan if the result is 1.0
            if resScore == 1.0:
                raw_input("Wanna execute?")
                self.execute_plan(arm_select)
                return True

            else:
                return False

        except rospy.ServiceException, e:
            print "Servie call failed: %s" % e

    def read_result(self, filename):
        # construct the folder name for the resulting trajectory library
        folder_sub = raw_input("please type in the folder your demo data being stored in: ")
        folder = "result_" + folder_sub

        with open(folder + "/" + filename, "r") as result:
            for i, l in enumerate(result):
                pass
            file_length = i + 1
            print("filelength", file_length)
        result.close()

        # get the resulting trajectory
        waypoints = []
        with open(folder + "/" + filename, "r") as result:
            for line in range(file_length):
                data = eval(result.readline())
                try:
                    # print("data", data)
                    waypoints.append(data)
                except:
                    pass
        result.close()

        # define the waypoints for testing purpose
        inputPoints = []
        for waypoint in waypoints:
            for item in waypoint:
                inputPoints.append(item)
        return inputPoints

    def read_result_new(self):

        with open("resultDMP_ginger_eeff.txt", "r") as result:
            for i, l in enumerate(result):
                pass
            file_length = i + 1
            print("filelength", file_length)
        result.close()

        # get the resulting trajectory
        waypoints = []
        with open("resultDMP_ginger_eeff.txt", "r") as result:
            for line in range(file_length):
                data = eval(result.readline())
                try:
                    # print("data", data)
                    waypoints.append(data)
                except:
                    pass
        result.close()

        # define the waypoints for testing purpose
        inputPoints = []
        for waypoint in waypoints:
            for item in waypoint:
                inputPoints.append(item)
        return inputPoints

    def get_init_angles(self, count):
        if count == 0:
            self.ginger.set_left_arm_mode(1)
            rospy.sleep(0.5)
            self.ginger.set_left_arm_joint([0, 0.2, 0.3, -0.78, 0, 0, 0], interpolation=1)
            rospy.sleep(0.5)

        # self.init_angles = [0, 0.2, 0.3, -0.78, 0, 0, 0]
        self.init_angles = self.ginger.get_left_arm_joint()
        print "initial angle is: ", self.init_angles
        self.ginger.set_left_arm_mode(4)
        rospy.sleep(0.5)
        temp = self.ginger.get_left_arm_EEF()
        self.init_pos.append(temp[1])
        self.init_pos.append(temp[0])

        # insert the home position to the top of the result file
        # self.init_pos = [[-0.1751, 0.3602, 0.0348, 0.9156], [-0.1185, -0.2340, 0.1451]]
        print "initial pos is: ", self.init_pos

        joints = []
        for item in self.init_pos[0]:
            joints.append(item)
        for item in self.init_pos[1]:
            joints.append(item)
        with open("resultDMP_ginger_eeff.txt", 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            f.write(str(joints))
            f.write("\n" + content)
        f.close()


if __name__ == "__main__":
    # Define the initial joint angles
    # initAngles_21 = [0.12059172666541951, 0.032427442409019125, 0.21557455513227075, -0.16147293197242885, -0.16147293197242885, 0.045181704745496765, -0.0014133836888979743]
    initAngles_73 = [0, 0.2, 0.3, -0.78, 0, 0, 0]
    # create a move_ginger object
    gin = Ginger()
    obj = MoveGinger(gin)
    # obj.get_init_angles()
    # initAngles = obj.init_angles
    file_name = "resultDMP_ginger_eeff_test.txt"
    file_name = "resultDMP_ginger_eeff_test_partially_failed.txt"
    inputPoints = obj.read_result(file_name)
    # inputPoints = obj.read_result_new()
    obj.new_ik_client(initAngles_73, inputPoints)

