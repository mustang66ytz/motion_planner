#!/usr/bin/env python
import rospy
import numpy as np
from robots.ginger import Ginger
from std_msgs.msg import Bool
import dmp_generator_ginger as dg
import dmp_utils
np.set_printoptions(precision=4)
import math
import tf
shut_down_signal = False


def new_target_generator(target, count):
    # this function generates more candidate targets based on the input target
    # get the delta angle to be added to the current target
    delta_angle = count * (2 * math.pi / 180)

    target_trans = target[-3:]
    target_rot = target[:4]
    target_mat = dmp_utils.quaternion_2_matrix(target_rot, target_trans)
    target_mat = dmp_utils.build_relative_rotation(target_mat, 0, 0, delta_angle)
    new_target_trans, new_target_rot = dmp_utils.matrix_2_quaternion(target_mat)
    new_target_rot = dmp_utils.normalize_quaternion(new_target_rot)
    new_target = new_target_rot.tolist() + new_target_trans

    return new_target


def target_generate(target):
    left_home_pos = dmp_utils.left_home_quaternion + dmp_utils.left_home_trans
    targets = []
    targets.append(left_home_pos)
    for point in target:
        targets.append(point)
    targets.append(left_home_pos)
    return targets


def callback(data, args):
    global shut_down_signal
    br = tf.TransformBroadcaster()

    while not shut_down_signal:
        if data.data:
            print "enter the dmp execution handler!"
            # get the target waypoints from the pickle file:
            #targets = dmp_utils.load_primitives_way_points()

            # get the target waypoints from a variable
            # define the target point here:
            lr = tf.TransformListener()
            mwc_left_targets = []
            lr.waitForTransform('/Back_Y', '/DMPTarget', rospy.Time(), rospy.Duration(0.1))
            (target_trans, target_rot) = lr.lookupTransform('/Back_Y', '/DMPTarget', rospy.Time(0))
            mwc_target = target_rot + target_trans
            mwc_left_targets.append(mwc_target)
            # generate the whole sequence of the targets
            targets = target_generate(mwc_left_targets)

            '''
            # offset the target
            for count, target in enumerate(targets):
                if not count == len(targets)-1:
                    targets[count] = dmp_utils.quaternion_2_matrix(target[:4], target[-3:])
                    targets[count] = target.dot(dmp_utils.build_relative_offset(0.08, 0, -0.04))
            '''
            # specify the execution choice, newik or oldik
            choice = args[3]
            # carry out the dmp execution
            #dg.move_primitives(choice, targets, args[0], args[1], args[2], args[4])

            result = False
            fail_count = 0
            current_target = None
            while not result:
                result, fail_point = dg.move_primitives(choice, targets, args[0], args[1], args[2], args[4])
                # change the target orientation if the current planning failed
                if not result:
                    fail_count = fail_count+1
                    targets[fail_point+1] = new_target_generator(targets[fail_point+1], fail_count)
                    current_target = targets[fail_point+1]
                if current_target is not None:
                    br.sendTransform(tuple(current_target[-3:]),
                                    tuple(current_target[:4]),
                                    rospy.Time.now(),
                                    '/Target_Modify',
                                    '/Back_Y')
            shut_down_signal = True
            print "program finished"


def dmp_generator(primitives_folder, robot, arm, ik_mode):
    ik_selection = "newik"
    rospy.Subscriber("dmp_execution", Bool, callback, (primitives_folder, arm, robot, ik_selection, ik_mode))
    # give the primitive library name where the demo data are stored
    print "here"
    rospy.spin()


if __name__ == "__main__":
    robot = Ginger()
    robot.rate.sleep()
    robot.set_left_hand_joint([-0.1]*5)
    # select an arm to control
    #arm = raw_input("Please enter the arm, left or right")
    arm = "left"
    folder = raw_input("Please enter the folder of the demo data")
    #ik_mode = raw_input("Enter the ik mode: 1 for normal ik mode")
    ik_mode = "1"
    dmp_generator(folder, robot, arm, ik_mode)
