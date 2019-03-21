import Queue
import operator
import threading

import numpy as np
from backups import april_tag_detection
import ginger_utils as utils

from yumipy import YuMiState


np.set_printoptions(precision=4)

import time

from robots.robotfactory import RobotFactory

from enum import Enum

class Robot(Enum):
    YUMI = 'yumi'
    DUMMY = 'blah'
    GINGER = 'ginger'

class VisualServoing(object):

    def __init__(self, robotType):
        # initiate an instance for Yumi robot
        self.yumiObj = RobotFactory.factory(robotType)

        if self.yumiObj is not None:
            self.yumiObj.get_hand("right").set_speed(self.yumiObj.get_robot().get_v(200))
            self.yumiObj.get_hand("left").set_speed(self.yumiObj.get_robot().get_v(200))
        else:
            err = 'Error: Robot not created'
            raise Exception(err)

########################## experiment ##########################################

left_pose, right_pose = Queue.Queue(5), Queue.Queue(5)
qr_pose_c = []
g_camera_base = None
calibration_singal = threading.Event()
trajectory_pose_c = np.array([[1, 0, 0, 0 * 100],
                                  [0, 1, 0, 0 * 100],
                                  [0, 0, 1, 0.6 * 100],
                                  [0, 0, 0, 1]]).dot(utils.rotation(np.pi/4, np.pi/4,np.pi/4)) #trajectory pose in camera frame
trajectory_pose_b = None # trajectory pose in base frame
trajectory_list_c = []
trajectory_list_b = []


def monitor(yc):
    global left_pose, right_pose
    while True:
        if left_pose.qsize() < 3:
            tmp = yc.get_pose('left')
            if tmp is not None:
                left_pose.put(tmp.matrix)
        if right_pose.qsize() < 3:
            tmp = yc.get_pose('right')
            if tmp is not None:
                right_pose.put(tmp.matrix)



def visualization(marker, apriltag):
    global g_camera_base
    global trajectory_pose_c
    global trajectory_pose_b
    global left_pose, right_pose
    global trajectory_list_b
    global trajectory_list_c
    global qr_pose_c

    from cameras.realsense import realsense
    camera = realsense()

    camera_params = camera.get_calibration()

    rotate = utils.rotation(0, 0.3 + np.pi / 2, -np.pi / 2)
    g_marker_left = np.array([[1, 0, 0, 0.075 * 100],
                              [0, 1, 0, 0 * 100],
                              [0, 0, 1, 0.015 * 100],
                              [0, 0, 0, 1]]).dot(rotate)

    calibration_singal.wait()
    time.sleep(0.1)

    while True:

        if not camera.get_frame():
            continue

        # print 'The queue size for left and right is : %d, %d' %(left_pose.qsize(), right_pose.qsize())
        g_camera_base, qr_pose = apriltag.marker_recognition(camera.color_image, camera_params, marker, g_marker_left=g_marker_left,
                                                    left_end_effector=left_pose.get(True),
                                                    right_end_effector=right_pose.get(True),
                                                    trajectory_pose_c = trajectory_pose_c,
                                                    trajectory_list_b = trajectory_list_b,
                                                    trajectory_list_c = trajectory_list_c)
        if qr_pose is not None:
            qr_pose_c.insert(0, qr_pose)
            if len(qr_pose_c) > 5:
                qr_pose_c = qr_pose_c[:5]


        trajectory_pose_b = np.linalg.pinv(g_camera_base).dot(trajectory_pose_c)
        trajectory_pose_b[:3, -1] = trajectory_pose_b[:3, -1] * 0.01


def manipulation(yc, mode):
    global g_camera_base
    global trajectory_pose_b
    global trajectory_pose_c
    global left_pose, right_pose
    global trajectory_list_b
    global trajectory_list_c
    global qr_pose_c
    yc.go_threading_home('left')
    yc.get_hand('left').close_gripper()

    yc.get_hand('left').goto_state(YuMiState([-76.67, -72.86, 22.74, 92.88, -32.53, 125.14, 87.63]))
    calibration_singal.set()

    ######################  MODE SLECTION #############################
    # SELECT THE MODE HERE
    MODE = mode
    ######################  MODE SLECTION #############################

    K = [1, 1, 1]
    K = [i * 0.2 for i in K]
    limit = 0.004 # meters
    tolerance = 0.005  # meters

    # Mode 1: follow a planned trajectory in base frame
    if MODE == 1:
        # raw_input('Stable the camera and press to start!!!')
        time.sleep(2)
        seg_points = 10
        trajectory_list_b = []
        tolerance = 0.005  # 5mm

        left_pose_trans = left_pose.get(True)[:3, -1]
        if trajectory_pose_b is None or left_pose_trans is None:
            print('No data recognized!')
        target_trans = trajectory_pose_b[:3, -1]
        print('target_trans is :', target_trans)

        # vector substract between target point (yellow point) in camera frame and left end-effector pose
        v = [((x1 - x2)/ seg_points).tolist() for (x1, x2) in zip(target_trans, left_pose_trans)]
        v = [v[0][0][0], v[1][0][0], v[2][0][0]]


        # add the segmeted point into trajectory
        for p in range(1, seg_points + 2):
            tmp = map(operator.add, left_pose_trans, [i * p for i in v])
            trajectory_list_b.append(tmp)


        for index in range(len(trajectory_list_b)):
            print(index)
            print(trajectory_list_b[index])

            # follow the current trajectory point
            while True:
                left_pose_translation = left_pose.get(True)
                e = [(x1 - x2).tolist() for (x1, x2) in zip(trajectory_list_b[index], left_pose_translation[:3, -1])]

                if all(np.abs(e) < tolerance):
                    index +=1
                    break
                u = [i * j for i, j in zip(K, e)]

                if np.abs(e[0]) < tolerance:
                    u[0] = 0
                if np.abs(e[1]) < tolerance:
                    u[1] = 0
                if np.abs(e[2]) < tolerance:
                    u[2] = 0

                for index_e, value_e in enumerate(e):
                    if u[index_e] > limit:
                        u[index_e] = limit
                    elif u[index_e] < -limit:
                        u[index_e] = -limit
                yc.move_delta('left', trans=u, rotation=None)
                # print(e)
                # print(u)
        print '------------ Finished --------------------'

    # Mode 2: real-time tracking
    if MODE == 2:

        while True:
            left_pose_translation = left_pose.get(True)[:3, -1]

            if trajectory_pose_b is None or left_pose_translation is None:
                continue
            e = [(x1 - x2).tolist() for (x1, x2) in zip(trajectory_pose_b[:3, -1], left_pose_translation)]
            e = [e[0][0][0], e[1][0][0], e[2][0][0]]

            if all(np.abs(e) < tolerance):
                continue

            u = [i * j for i, j in zip(K, e)]

            if np.abs(e[0]) < tolerance:
                u[0] = 0
            if np.abs(e[1]) < tolerance:
                u[1] = 0
            if np.abs(e[2]) < tolerance:
                u[2] = 0

            for index_e, value_e in enumerate(e):
                if u[index_e] > limit:
                    u[index_e] = limit
                elif u[index_e] < -limit:
                    u[index_e] = -limit
            yc.move_delta('left', trans=u, rotation=None)
            # print ('-----------', u, e)


    # Mode 3: planned trajectory purely using the camera frame
    if MODE == 3:
        time.sleep(3)

        seg_points = 10
        trajectory_list_c = []

        qr_pose = qr_pose_c[-1]
        if qr_pose is None:
            print('No data recognized!')

        v = [((x1 - x2) / seg_points).tolist() for (x1, x2) in zip(trajectory_pose_c[:3, -1], qr_pose[:3, -1])]
        v = [v[0][0][0], v[1][0][0], v[2][0][0]]

        for p in range(1, seg_points + 1):
            tmp = map(operator.add,  qr_pose[:3, -1], [i * p for i in v])
            trajectory_list_c.append(tmp)

        for index in range(len(trajectory_list_c)):
            print(index)
            print(trajectory_list_c[index])

            while True:
                qr_translation = qr_pose_c[-1][:3, -1].tolist()
                qr_translation = [qr_translation[0][0], qr_translation[1][0], qr_translation[2][0]]
                cur_target = [trajectory_list_c[index][0][0][0,0], trajectory_list_c[index][1][0][0,0], trajectory_list_c[index][2][0][0,0]]

                e_c = [(x1 - x2).tolist() * 0.01 for (x1, x2) in zip(cur_target, qr_translation)]
                if all(np.abs(e_c) < tolerance):
                    break

                K2 = [1, 1, 1]
                K2 = [i * -0.1 for i in K2]

                u = [i * j for i, j in zip(K2, e_c)]

                for index_e, value_e in enumerate(e_c):
                    if u[index_e] > limit:
                        u[index_e] = limit
                    elif u[index_e] < -limit:
                        u[index_e] = -limit
                yc.move_delta('left', trans=u, rotation=None)
                print('+', u, e_c)



if __name__ == '__main__':
    rotate = utils.rotation(0, np.pi / 8 + np.pi / 2, -np.pi / 2)

    g_m_left = np.array([[1, 0, 0, 0.083 * 100],
                        [0, 1, 0, 0 * 100],
                        [0, 0, 1, 0.015 * 100],
                        [0, 0, 0, 1]]).dot(rotate)


    try:
        yc = VisualServoing(Robot.YUMI.value)
        apriltag = april_tag_detection.ApriltagDetection()
        marker = {2: 3.47}

        monitor1 = threading.Thread(target=monitor, args=(yc.yumiObj,), name='pose monitor')
        v1 = threading.Thread(target=visualization, args=(marker, apriltag,), name='visualization')
        m1 = threading.Thread(target=manipulation, args=(yc.yumiObj, 2,), name='manipulation')


        monitor1.start()
        m1.start()
        v1.start()

        v1.join()
        m1.join()
        monitor1.join()

    except Exception as error:
        print('Caught this error: ' + repr(error))

