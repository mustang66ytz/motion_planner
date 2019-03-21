#!/usr/bin/env python
from robots.ginger import Ginger
# from camera import Camera
import numpy as np
import tf
# import utils
import threading

if __name__ == '__main__':
    robot = Ginger()
    # camera = Camera()

    robot.rate.sleep()

    Fall = True

    if Fall:
        robot.set_main_body_mode(1)
        robot.set_left_hand_mode(1)
        robot.set_right_hand_mode(1)
        robot.set_left_arm_mode(1)
        robot.set_right_arm_mode(1)

        robot.go_zero()
        # robot.set_left_arm_joint([0]*7, interpolation=None, send_times=)

    robot.open_gripper('left')
    #
    #     #
#     for index, each_joint in enumerate(reversed(first_segmentation)):
#         robot.set_left_arm_joint(each_joint, interpolation=None, send_times=5)
#         robot.rospy.sleep(.6)
#     robot.set_main_body_joint([0, 0, 0, 0, 0, 0, 0], interpolation=None, send_times=15)
#
#     for index, each_joint in enumerate(first_segmentation):
#         if index == 0:
#             robot.set_left_arm_joint(each_joint, interpolation=None, send_times=20)
#             robot.rospy.sleep(1.5)
#         else:
#             robot.set_left_arm_joint(each_joint, interpolation=None, send_times=5)
#             robot.rospy.sleep(.7)
#     robot.rospy.sleep(2)

    #
    # while True:
    #     pass