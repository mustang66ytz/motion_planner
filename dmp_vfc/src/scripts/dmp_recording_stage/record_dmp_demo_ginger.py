#!/usr/bin/env python
import numpy as np
import cv2
import time
import os
from robots.ginger import Ginger

class Recorder(object):
    def __init__(self, gin):
        self.flag = False # keeps track of the recoeding state
        self.listOfJoints = []
        self.startTime = None
        self.endTime = None
        self.recordedData = None
        self.ginger = gin

    def record(self, mode, arm):
        """
        :param mode:
        :return:
        """
        if mode == "joint":
            if arm == "left":
                data = self.ginger.get_left_arm_joint()
            elif arm == "right":
                data = self.ginger.get_right_arm_joint()
            else:
                data = None
            self.listOfJoints.append(data)
            self.recordedData.write(str(data))
            self.recordedData.write("\n")

        elif mode == "ee":
            # convert the position into a 7D data (4 quaternion/rotations + 3 positions)
            if arm == "left":
                data = self.ginger.get_left_arm_EEF()
            elif arm == "right":
                data = self.ginger.get_right_arm_EEF()
            else:
                data = None

            print "data now: ", data
            # rotation is the quaternion
            rotation = data[1]
            position = data[0]
            final_data = rotation+position # final_data = [ rotation(4) + position(3)]
            print final_data
            self.listOfJoints.append(final_data)
            self.recordedData.write(str(final_data))
            self.recordedData.write("\n")

        else:
            print "wrong mode, choose from 'joint' or 'ee'"
            return

    def listener(self, mode, arm):
        foldername = raw_input("provide foldername for motion primitives: ")

        # cd to the motion_primitives folder:
        orig_dir = os.getcwd()
        os.chdir("../dmp_learning_primitives")

        # make the motion primitive directory folder
        if not os.path.exists(foldername):
            os.mkdir(foldername)
            print("Directory ", foldername, " Created ")
        else:
            print("Directory ", foldername, " already exists")

        # exit back to the original directory
        os.chdir(orig_dir)

        print "start listening any keystroke, press space to start recording, and press 'q' to quit!"

        # initializae the loop
        count = 0
        self.flag = False
        while True:
            m = cv2.imread('../pics/instruction.png')
            cv2.imshow("test", m)
            k = cv2.waitKey(1)
            # start recording when space is pressed
            if not self.flag and (k & 0xFF == ord(' ')):
                self.flag = True

                #open a file
                filename = "R_" + str(count)
                self.recordedData = open("../dmp_learning_primitives/" + foldername + "/" + filename, 'w')
                print filename + "opened"

            # record data for opened file
            while self.flag:

                # display HCI
                cv2.imshow("test", m)
                k = cv2.waitKey(1)

                # record
                time.sleep(0.2)
                print filename + "recording"
                self.record(mode, arm)

                #check for finish recording
                if k & 0xFF == ord(' '):
                    print "exiting"
                    self.flag = False
                    self.recordedData.close()
                    print filename + "closed"
                    count += 1
                    break

                # exiting program
                if k & 0xFF == ord('q'):
                    print "exiting program"
                    self.flag = False
                    self.recordedData.close()
                    print filename + "closed"
                    return True

            # exiting program
            if k & 0xFF == ord('q'):
                print "exiting program"
                self.flag = False
                return True
        # return

if __name__ == '__main__':
    gin = Ginger()
    target_arm = "left" # choose from left and right
    # customize the dmp mode: "ee" for end-effector, and "joint" for joint space
    mode = "ee"
    # initialize a recorder
    new_recorder = Recorder(gin)
    new_recorder.listener(mode, target_arm)

