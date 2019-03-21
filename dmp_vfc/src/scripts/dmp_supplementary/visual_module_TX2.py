#!/usr/bin/env python
import numpy as np
import tf
import math
import rospy
import april_tag_detection
from robots.ginger import Ginger
from dmp.msg import *
import cv2
import ginger_utils
np.set_printoptions(precision=4)

lr = None # tf listener
br = None # tf broadcaster
marker_pose_2D = None # marker position in 2d image

def quaternion_2_matrix(quat):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    res = np.zeros((4, 4))
    res[0] = 1-2*(y*y+z*z)
    res[1] = 2*(x*y-w*z)
    res[2] = 2*(x*z+w*y)
    res[4] = 2*(x*y+w*z)
    res[5] = 1-2*(x*x + z*z)
    res[6] = 2*(y*z-w*x)
    res[8] = 2*(x*z-w*y)
    res[9] = 2*(y*z+w*x)
    res[10] = 1-2*(x*x+y*y)
    res[15] = 1


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

def normalize_quaternion(quaternion):
    if not len(quaternion) == 4:
        return None
    norm = np.linalg.norm(quaternion)
    temp = []
    temp.append(quaternion[0]/norm)
    temp.append(quaternion[1]/norm)
    temp.append(quaternion[2]/norm)
    temp.append(quaternion[3]/norm)
    return temp

def build_rotation(axis, angle):
    rotation = None
    if axis == 'x':
        rotation = np.matrix([[1, 0, 0, 0],
                             [0, math.cos(-angle), math.sin(-angle), 0],
                             [0, -math.sin(-angle), math.cos(-angle), 0],
                             [0, 0, 0, 1]])
    if axis == 'y':
        rotation = np.matrix(
            [[math.cos(-angle), 0, -math.sin(-angle), 0],
             [0, 1, 0, 0],
             [math.sin(-angle), 0, math.cos(-angle), 0],
             [0, 0, 0, 1]]
        )
    if axis == 'z':
        rotation = np.matrix(
            [[math.cos(-angle), math.sin(-angle), 0, 0],
             [-math.sin(-angle), math.cos(-angle), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        )
    return rotation

def get_picture_point(img, target_pose, camera):
    axis = np.float32([[0, 0, 0]]).reshape(-1, 3)
    fx, fy, cx, cy = camera.get_calibration()
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    rvec, _ = cv2.Rodrigues(target_pose[:3, :3])
    tvec = target_pose[:3, 3]
    dcoeffs = np.zeros(5)
    po, _ = cv2.projectPoints(axis, rvec, tvec, K, dcoeffs)
    pts = np.round(po).astype(int)
    pts = pts.reshape(-1, 2)
    return pts


def build_relative_offset(x_off, y_off, z_off):
    return np.matrix([[1, 0, 0, 0.03+x_off],
                      [0, 1, 0, 0+y_off],
                      [0, 0, 1, 0+z_off],
                      [0, 0, 0, 1]])


def build_absolute_offset(original, x_off, y_off, z_off):
    new = []
    new.append(original[0]+x_off)
    new.append(original[1]+y_off)
    new.append(original[2]+z_off)
    return new


def build_relative_rotation(original, x_rot, y_rot, z_rot):
    rotatex = build_rotation('x', x_rot)
    rotatey = build_rotation('y', y_rot)
    rotatez = build_rotation('z', z_rot)
    # apply the rotations here
    return original.dot(rotatex).dot(rotatey).dot(rotatez)


def visualization(camera, visual_module, markers, g_camera_head):
    global marker_pose_2D
    global lr, br

    # this is the transformation from left end-effector to the left hand, only the translation needed
    track_marker_list = []

    # tune the parameters below for the relative offset
    g_marker_left = build_relative_offset(0, 0, 0)

    kalman_count = {}
    for tid in track_marker_list:
        kalman_count[tid] = 0

    target_tid = 2

    if not target_tid in markers:
        exit(0)

    # define necessary ros publishers to publish corresponding ros msgs
    marker_pose_pub = rospy.Publisher('marker_position', Waypoint, queue_size=10)
    marker_pose_eef_pub = rospy.Publisher('marker_position_eef', Waypoint, queue_size=10)

    while not rospy.is_shutdown():
        # get camera frames
        if not camera.get_frame():
            continue

        marker_pose, _ = visual_module.marker_recognition(camera.color_image)
        # visualization
        overlay = camera.color_image.copy()
        overlay = overlay + visual_module.dimg
        visual_module.draw_markers(overlay)

        # stopping conditions and clean ups for kalman filter
        for tid in track_marker_list:
            kalman_count[tid] += 1
            if kalman_count[tid] > 200:
                visual_module.remove_tid(tid)
                kalman_count[tid] = 0

        trans1, qua1 = matrix_2_quaternion(g_camera_head)
        br.sendTransform(tuple(trans1),
                         tuple(qua1),
                         rospy.Time.now(),
                         '/Camera',
                         '/Head')

        if target_tid in marker_pose:
            point = get_picture_point(camera.color_image, marker_pose[target_tid], camera)
            marker_pose_2D = point
            cur_marker = marker_pose[target_tid]
            rotate1 = build_rotation('z', -0.5*math.pi)
            rotate2 = build_rotation('x', math.pi)
            # apply the rotations here
            cur_marker = cur_marker.dot(rotate1).dot(rotate2)
            # this is for tid = 2
            #cur_marker = build_relative_rotation(cur_marker, 0, -0.3*math.pi, 0)
            # this is for tid = 5
            cur_marker = build_relative_rotation(cur_marker, 0.5*math.pi, 0.5*math.pi, 0.25*math.pi)
            trans4, qua4 = matrix_2_quaternion(cur_marker)

            # normalize the transferred quaternion
            qua4 = normalize_quaternion(qua4)
            br.sendTransform(tuple(trans4),
                             tuple(qua4),
                             rospy.Time.now(),
                             '/Marker',
                             '/Camera')
        try:
            # find the transformation from marker to back_y, since all the dmp end-effector tfs are respective to back_y
            lr.waitForTransform('/Back_Y', '/Marker', rospy.Time(), rospy.Duration(0.1))
            (trans3, rot3) = lr.lookupTransform('/Back_Y', '/Marker', rospy.Time(0))
            cur_marker = cur_marker.dot(build_relative_offset(0.08, 0, -0.03))
            # offset the absolute marker translation for the coffee picking task:
            #trans3 = build_absolute_offset(trans3, -0.2, 0, 0)
            marker_pose_back = rot3 + trans3
            # obtain the marker tf respect to the back_y
            # send the targeting waypoint pos as a ros msg to the rostopic
            marker_pose_back_msg = Waypoint()
            marker_pose_back_msg.target = marker_pose_back
            marker_pose_pub.publish(marker_pose_back_msg)
        except:
            print "No marker detected, place the marker"
        try:
            endeff_pose_back_matrix = ginger_utils.quaternion_2_matrix(trans3, rot3)#.dot(g_marker_left)
            endeff_pose_back_trans, endeff_pose_back_qua = matrix_2_quaternion(endeff_pose_back_matrix)
            endeff_pose_back_qua = normalize_quaternion(endeff_pose_back_qua)
            endeff_pose_back = endeff_pose_back_qua+endeff_pose_back_trans
            # send the targeting waypoint for the end-effector through a ros msg
            endeff_pose_back_msg = Waypoint()
            endeff_pose_back_msg.target = endeff_pose_back
            marker_pose_eef_pub.publish(endeff_pose_back_msg)
        except:
            print "targeting end-effector position getter failed"
        # rendering
        cv2.namedWindow('marker.png', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('marker.png', 960, 960)
        cv2.imshow('marker.png', overlay)
        cv2.waitKey(1)


if __name__ == '__main__':
    global lr, br
    # then define the marker id for the object
    markers = {0: 3.9, 6: 2.5, 2: 2.9, 5: 1.9, 20: 3.0}
    avm = april_tag_detection.ApriltagVisionModule(markers)

    # select an appropriate camera first
    camera_selection = 3
    cam = None
    if camera_selection == 0:
        pass
    elif camera_selection == 1:
        from cameras.realsense import realsense
        cam = realsense()
    elif camera_selection == 2:
        from cameras.web_camera import web_camera
        cam = web_camera()
    elif camera_selection == 3:
        from cameras.realsense_TX2 import realsenseTX2
        cam = realsenseTX2()

    # then select a robot
    rospy.init_node("visualrender", anonymous=True)
    #robot = Ginger()
    #robot.rate.sleep()

    _GINGER = 'BUTTERFLY'
    g_camera_head = None
    if _GINGER == 'OLD':
        g_camera_head = tf.transformations.euler_matrix(0, - np.pi / 5.5, np.pi / 2, 'syxz')  # y-x-z axis
        g_camera_head[0, -1] = 0.14
        g_camera_head[1, -1] = 0
        g_camera_head[2, -1] = 0.08

    elif _GINGER == 'NEW':
        g_camera_head = tf.transformations.euler_matrix(0, -np.pi / 8, np.pi / 2, 'syxz')  # y-x-z axis
        g_camera_head[0, -1] = 0.3
        g_camera_head[1, -1] = 0
        g_camera_head[2, -1] = 0.08

    elif _GINGER == 'BUTTERFLY':
        K = [-0.4, 0.7]
        g_camera_head = tf.transformations.euler_matrix(0, -np.pi / 4.5, np.pi / 2, 'syxz')  # y-x-z axis
        g_camera_head[0, -1] = 0.36
        g_camera_head[1, -1] = 0.03
        g_camera_head[2, -1] = 0.00  # -0.25

    # define the tf listener and broadcaster after ros node initialization
    lr = tf.TransformListener()
    br = tf.TransformBroadcaster()
    visualization(cam, avm, markers, g_camera_head)