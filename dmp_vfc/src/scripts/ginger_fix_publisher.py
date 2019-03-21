#!/usr/bin/env python

import rospy
import tf
import ginger_utils
import numpy as np

broadcaster = None
listener = None

def send_fix_tf(g_camera_head):
    try:
        trans1, qua1 = ginger_utils.matrix_2_quaternion(g_camera_head)

        broadcaster.sendTransform(tuple(trans1),
                                  tuple(qua1),
                                  rospy.Time.now(),
                                  '/Camera',
                                  '/Head')
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo('Not successfully send camera-head tf topics')


def compensation():
    try:

        pose = listener.lookupTransform('/LeftEndEffector', '/Wrist_visual', rospy.Time(0))
        trans = pose[0]

        broadcaster.sendTransform(tuple(trans),
                                  tuple([0, 0, 0, 1]),
                                  rospy.Time.now(),
                                  '/Virtual_EEF',
                                  '/LeftEndEffector')

        rospy.sleep(.02)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

if __name__ == '__main__':
    rospy.init_node('ginger_fixed_tf_publisher')

    rospy.loginfo('Ginger fixed parametere adjusting..')
    rospy.loginfo('Ginger compensated parametere...')

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    g_camera_head = tf.transformations.euler_matrix(0, -np.pi / 6.7, np.pi / 2, 'syxz')  # y-x-z axis
    g_camera_head[0, -1] = 0.14
    g_camera_head[1, -1] = 0.00
    g_camera_head[2, -1] = 0.12  # -0.25

    while not rospy.is_shutdown():
        send_fix_tf(g_camera_head)
        rospy.sleep(0.05)
        # compensation()