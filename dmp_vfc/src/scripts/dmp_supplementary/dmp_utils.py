#!/usr/bin/env python
import numpy as np
import rospy
import pickle
import tf
np.set_printoptions(precision=4)
import math

# this script defines some general utility functions to be used by dmp module

# input: rotational axis, rotational angle
# output: a 4by4 rotational martrix
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

# input: 4by4 rotational matrix needed to be rotated, rotational angles about x, y, and z axes
# output: the rotated 4by4 rotational matrix
def build_relative_rotation(original, x_rot, y_rot, z_rot):
    rotatex = build_rotation('x', x_rot)
    rotatey = build_rotation('y', y_rot)
    rotatez = build_rotation('z', z_rot)
    # apply the rotations here
    return original.dot(rotatex).dot(rotatey).dot(rotatez)

# input: a 4by4 rotational matrix
# output: 1by3 translation and 1by4 quaternion
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

# input: 1by4 quaternion and 1by3 translation
# output: 4by4 rotational matrix
def quaternion_2_matrix(quat, trans):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    res = np.zeros((4, 4))
    res[0][0] = 1-2*(y*y+z*z)
    res[0][1] = 2*(x*y-w*z)
    res[0][2] = 2*(x*z+w*y)
    res[0][3] = trans[0]
    res[1][0] = 2*(x*y+w*z)
    res[1][1] = 1-2*(x*x + z*z)
    res[1][2] = 2*(y*z-w*x)
    res[1][3] = trans[1]
    res[2][0] = 2*(x*z-w*y)
    res[2][1] = 2*(y*z+w*x)
    res[2][2] = 1-2*(x*x+y*y)
    res[2][3] = trans[2]
    res[3][3] = 1

    return res

# input: relative offset to the target's local coordinate in meters in x, y, and z direction
# output: a 4by4 matrix
def build_relative_offset(x_off, y_off, z_off):
    return np.matrix([[1, 0, 0, 0.03+x_off],
                      [0, 1, 0, 0+y_off],
                      [0, 0, 1, 0+z_off],
                      [0, 0, 0, 1]])


#input: 4by4 matrix, and offset in x, y, and z directions
#output: offseted 4by4 matrix
def build_absolute_offset(original, x_off, y_off, z_off):
    new = []
    new.append(original[0]+x_off)
    new.append(original[1]+y_off)
    new.append(original[2]+z_off)
    return new


# input: the original quaternion
# output: the normalized quaternion
def normalize_quaternion(quat):
    norm = np.linalg.norm(quat)
    quat[0] = quat[0] / norm
    quat[1] = quat[1] / norm
    quat[2] = quat[2] / norm
    quat[3] = quat[3] / norm
    return quat


# input: 1d eef list
# output: 2d list : each row is a waypoint's pose
def convert1dto2d(traj):
    temp = []
    waypt2d = []
    for count, item in enumerate(traj):
        if (count + 1) % 7 == 0 and not count == 0:
            temp.append(item)
            waypt2d.append(temp)
            temp = []
        else:
            temp.append(item)
    print "2d waypoints: ", waypt2d
    return waypt2d


# input: relative file path
# output: a list
def load_pickle_file(filename):
    pickle_off = open(filename, "rb")
    result = pickle.load(pickle_off)
    pickle_off.close()
    return result

# this load the generated dmp way points from the pickle file
def load_primitives_way_points():
    primitive_waypoint_pickle = load_pickle_file("../waypoint_targets/mp_way_points.pickle")
    return primitive_waypoint_pickle

# Following are some poses for ginger
#left_home_trans = [-0.1856695794148949, -0.2124832623176604, -0.016995409634506024]
#left_home_quaternion = [0.00022654249216508434, -0.0005469729404355046, 0.00017302118047832443, 0.9999998210313703]
#right_home_trans = [-0.18567598453464002, 0.21242531723787053, -0.017007879232618376]
#right_home_quaternion = [1.754584572893577e-05, 0.0007713534482236648, -0.0004400999228857989, 0.9999996167589579]
# here is the home position for the non-singularity
#left_home_trans = [-0.15882225403939296, -0.21258041893974167, 0.08809180992395889]
#left_home_quaternion = [3.7469323037635316e-06, 0.24741863338856093, -2.303780439016265e-05, 0.9689086860005726]
#left_home_trans = [-0.1683553984222372, -0.21257995604742347, 0.06835740934122332]
#left_home_quaternion = [2.4258882016028687e-06, 0.19867185077607952, -2.3398552315110973e-05, 0.9800660782089282]
# here is the home position for the non-singularity human-like ready gesture
left_home_trans = [-0.1185, -0.2340, 0.1451]
left_home_quaternion = [-0.1751, 0.3602, 0.0348, 0.9156]
right_home_trans = [-0.15882476394746625, 0.21257830005049894, 0.0880896534415361]
right_home_quaternion = [-1.256458299953984e-05, 0.2474087286666777, 3.018587555425712e-05, 0.9689112149265698]

right_home_pos = right_home_quaternion+right_home_trans
left_home_pos = left_home_quaternion+left_home_trans
# hard-coded target position for testing purpose
# this test case data is from the R_0 demo data, it is good
#test_goal_left = [-0.5571638192069343, 0.575242166261965, -0.21679645766259464, 0.5582689768911218, 0.22445238346047316, -0.20288282629715598, 0.3291512751673038]
# this test case data is from the result.txt, successful
#test_goal_left = [-0.5737915400767383, 0.5483960625760551, -0.047160271910669675, 0.5261555836118438, 0.12876627483611178, -0.10700795949092401, 0.35237175901101414]
# this test case data is artificial, successful
#test_goal_left_trans = [0.12876627483611178-0.1, -0.10700795949092401, 0.35237175901101414]
'''
# test data on Jan 17:
test_goal_left_trans = [0.1, -0.3, 0.25] # successful
test_goal_left_trans = [0.1, -0.3, 0.35] # successful
test_goal_left_trans = [0.1, -0.1, 0.25] # successful
test_goal_left_trans = [0.1, -0.1, 0.35] # successful
test_goal_left_trans = [0.1, -0.2, 0.3] # successful
test_goal_left_trans = [0.1, -0.2, 0.25] # fail
test_goal_left_trans = [0.1, -0.2, 0.26] # successful
test_goal_left_trans = [0.1, -0.2, 0.35] # successful
test_goal_left_trans = [0.1, -0.1, 0.3] # successful
test_goal_left_trans = [0.1, -0.3, 0.3] # successful, mp6 much better than mp3
'''
test_goal_left_rot = [-0.6337364484036944, 0.6820241366752285, -0.16401977835833179, 0.3236558038784964]
test_goal_left_trans = [0.1, -0.3, 0.3] # successful, mp6 much better than mp3
test_goal_right = []


