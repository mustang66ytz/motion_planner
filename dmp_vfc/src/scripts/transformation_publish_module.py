#!/usr/bin/env python
import rospy
import tf
import ginger_utils
import numpy as np

def send_fix_tf(g_camera_head):
    broadcaster = tf.TransformBroadcaster()

    broadcaster.sendTransform(tuple([-0.09, 0.095, 0]),
                              tuple([0, 0, 0, 1]),
                              rospy.Time.now(),
                              '/Left_hand',
                              '/LeftEndEffector')

    broadcaster.sendTransform(tuple([-0.13, 0.05, 0]),
                              tuple([0, 0, 0, 1]),
                              rospy.Time.now(),
                              '/Right_hand',
                              '/RightEndEffector')

    trans1, qua1 = ginger_utils.matrix_2_quaternion(g_camera_head)

    broadcaster.sendTransform(tuple(trans1),
                              tuple(qua1),
                              rospy.Time.now(),
                              '/Camera',
                              '/Head')


if __name__ == '__main__':
    rospy.init_node('transformation_publish_node')

    rospy.loginfo('Ginger parameters adjusting..')

    _GINGER = 'NEW'
    # _GINGER = 'OLD'

    #############################
    if _GINGER == 'OLD':
        K = [-0.4, -0.3]

        g_camera_head = tf.transformations.euler_matrix(0, - np.pi / 5.5, np.pi / 2, 'syxz')  # y-x-z axis
        g_camera_head[0, -1] = 0.14
        g_camera_head[1, -1] = 0
        g_camera_head[2, -1] = 0.08  # -0.25

    elif _GINGER == 'NEW':
        K = [-0.4, 0.7]
        g_camera_head = tf.transformations.euler_matrix(0, -np.pi / 8, np.pi / 2, 'syxz')  # y-x-z axis
        g_camera_head[0, -1] = 0.3
        g_camera_head[1, -1] = 0
        g_camera_head[2, -1] = 0.08  # -0.25

    while True:
        send_fix_tf(g_camera_head)