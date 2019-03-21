#!/usr/bin/env python
import math
import tf
import numpy as np
import rospy
import pickle
from dmp_vfc.msg import *
from std_msgs.msg import Bool
import dmp_utils
np.set_printoptions(precision=4)

# do some rotations on the testing data here:
test_goal_left_matrix = dmp_utils.quaternion_2_matrix(dmp_utils.test_goal_left_rot, dmp_utils.test_goal_left_trans)
test_goal_left_matrix = dmp_utils.build_relative_rotation(test_goal_left_matrix, math.pi/4, math.pi/24, math.pi/4)
test_goal_left_trans, test_goal_left_rot = dmp_utils.matrix_2_quaternion(test_goal_left_matrix)
test_goal_left = test_goal_left_rot.tolist()+test_goal_left_trans
print "test_goal_left is: ", test_goal_left
test_goal_right = dmp_utils.test_goal_right

# insert the home pose as the initial pose for dmp:
recordedDataRight = []
recordedDataLeft = []
right_home_pos = dmp_utils.right_home_quaternion+dmp_utils.right_home_trans
left_home_pos = dmp_utils.left_home_quaternion+dmp_utils.left_home_trans
recordedDataRight.append(right_home_pos)
recordedDataLeft.append(left_home_pos)

# declare the target poses for the mwc demo purpose:
mwc_left_targets = []
mwc_left_targets_rot = [-0.16423, 0.75264, 0.010383, 0.63754]
mwc_left_targets_trans = [0.16535, -0.28777, 0.27845]
mwc_left_targets.append(mwc_left_targets_rot+mwc_left_targets_trans)
mwc_left_targets.append(left_home_pos)
mwc_right_targets = []
data_command = ""

def callback(data, args):
    foldername = "../waypoint_targets"
    dmp_exe_pub = rospy.Publisher('dmp_execution', Bool, queue_size=10)
    arm = args
    new_marker_pose = []
    global data_command

    while True:
        print "press 'r' to use the data from the vision"
        print "press 't' to use predefined target pose"
        print "press 'm' to use predefined target pose for mwc demo"
        print "press 'q' to exit, and 'e' to execute the DMP plan"
        command = raw_input()
        if command == 't':
            print "-----------using the testing data-------------------------"
            data_command = "t"
            if arm == 'left':
                recordedDataLeft.append(test_goal_left)
                recordedDataLeft.append(left_home_pos)
            if arm == 'right':
                recordedDataRight.append(test_goal_right)
                recordedDataRight.append(right_home_pos)

        if command == 'm':
            print "-----------using the mwc demo data------------------------"
            data_command = "m"
            if arm == 'left':
                for target_left in mwc_left_targets:
                    recordedDataLeft.append(target_left)
            if arm == 'right':
                for target_right in mwc_right_targets:
                    recordedDataRight.append(target_right)

        if command == 'r':
            print "-----------recording the current marker position----------"
            data_command = "r"
            print data.target

            # this is the final end-effector pose (end for linear interpolation)
            cur_marker = dmp_utils.quaternion_2_matrix(data.target[:4], data.target[-3:])
            cur_marker = cur_marker.dot(dmp_utils.build_relative_offset(0.11, -0.025, 0.04))
            trans, qua = dmp_utils.matrix_2_quaternion(cur_marker)
            final_eef_pose = qua.tolist() + trans


            # offset the marker for linear interoplation beginning pose
            cur_marker_offset = cur_marker.dot(dmp_utils.build_relative_offset(0.04, -0.02, 0.0))
            trans_offset, qua_offset = dmp_utils.matrix_2_quaternion(cur_marker_offset)
            semi_final_eef_pose = qua_offset.tolist() + trans_offset

            if arm == 'left':
                recordedDataLeft.append(semi_final_eef_pose)
                recordedDataLeft.append(left_home_pos)
            if arm == 'right':
                recordedDataRight.append(semi_final_eef_pose)
                recordedDataRight.append(right_home_pos)

        if command == 'q':
            print "----------------------finish recording--------------------"
            pickling_on = open(foldername+"/mp_way_points.pickle", "wb")
            if arm == 'left':
                pickle.dump(recordedDataLeft, pickling_on)
            if arm == 'right':
                pickle.dump(recordedDataRight, pickling_on)
            pickling_on.close()

        if command == 'e':
            print "---------------------executing DMPs------------------------"
            while not rospy.is_shutdown():
                dmp_exe_pub.publish(True)
                rospy.sleep(0.2)
                br = tf.TransformBroadcaster()
                # render the target poses in rviz:
                if data_command == "r":
                    br.sendTransform(tuple(data.target[-3:]),
                                     tuple(data.target[:4]),
                                     rospy.Time.now(),
                                     '/Target1',
                                     '/Back_Y')

                    br.sendTransform(tuple(final_eef_pose[-3:]),
                                        tuple(final_eef_pose[:4]),
                                        rospy.Time.now(),
                                        '/LinearTarget',
                                        '/Back_Y')

                    br.sendTransform(tuple(semi_final_eef_pose[-3:]),
                                     tuple(semi_final_eef_pose[:4]),
                                     rospy.Time.now(),
                                     '/DMPTarget',
                                     '/Back_Y')

                if  data_command == "t":
                    br.sendTransform(tuple(test_goal_left_trans),
                                     tuple(test_goal_left_rot),
                                     rospy.Time.now(),
                                    '/Target',
                                    '/Back_Y')

                if data_command == "m":
                    br.sendTransform(tuple(mwc_left_targets_trans),
                                     tuple(mwc_left_targets_rot),
                                     rospy.Time.now(),
                                     '/DMPTarget',
                                     '/Back_Y')


def listener(arm):
    print "start listening any keystroke, press space to record a motion way point"
    rospy.Subscriber("marker_position_eef", Waypoint, callback, arm)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('dmp_waypoint_listener')
    arm = "left"
    listener(arm)