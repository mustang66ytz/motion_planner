#!/usr/bin/env python

import rospy
import cv2
import tf
import ginger_utils
import numpy as np
from april_tag_detection import ApriltagVisionModule
from sensor_msgs.msg import CompressedImage


def send_tf(marker_pose_dic):
    broadcaster = tf.TransformBroadcaster()

    for key, value in marker_pose_dic.items():
        marker_name = 'Marker_' + str(key)
        translation, quaternion = ginger_utils.matrix_2_quaternion(value)

        quaternion = quaternion / np.linalg.norm(quaternion)


        broadcaster.sendTransform(tuple(translation),
                                  tuple(quaternion),
                                  rospy.Time.now(),
                                  marker_name,
                                  '/Camera')

        if key == 62:
            broadcaster.sendTransform(tuple([-0.047, 0.086, 0.005]),
                                      tuple([0, 0, 0, 1]),
                                      rospy.Time.now(),
                                      '/Thumb',
                                      marker_name)

            broadcaster.sendTransform(tuple([0.025, 0.104, 0.037]),
                                      tuple([0, 0, 0, 1]),
                                      rospy.Time.now(),
                                      '/Index',
                                      marker_name)

            broadcaster.sendTransform(tuple([-0.001, 0.036, 0.007]),
                                      tuple([0, 0, 0, 1]),
                                      rospy.Time.now(),
                                      '/Jaw',
                                      marker_name)

            broadcaster.sendTransform(tuple([-0.006, 0.062, -0.015]),
                                      tuple([0, 0, 0, 1]),
                                      rospy.Time.now(),
                                      '/Center',
                                      marker_name)



            broadcaster.sendTransform(tuple( [-0.011, -0.068, 0.019]),
                                      tuple([0, 0, 0, 1]),
                                      rospy.Time.now(),
                                      '/Wrist_visual',
                                      marker_name)


        # Grasp point from bottle
        if key == 31:
            broadcaster.sendTransform(tuple([0, 0, 0.02]),
                                      tuple([0, 0, 0, 1]),
                                      rospy.Time.now(),
                                      '/Grasp_point',
                                      marker_name)


if __name__ == '__main__':
    rospy.init_node('vision_tf_publisher')

    markers = {62:2.05, 31:2.05, 30:1.9, 1:4}
    cam = None
    avm = ApriltagVisionModule(markers)

    image_pub = rospy.Publisher("/debug/image_raw/compressed",CompressedImage, queue_size=5)
    
    ################# camera selection parameter #################################

    # 0: video samples (TODO)
    # 1: librealsense
    # 2: webcam
    # 3: kinect2 through ROS (TODO)
    camera_selection = 3

    if camera_selection == 0:
        # video loading
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

    ################# camera selection parameter #################################

    kalman_count = {}

    for tid in markers:
        kalman_count[tid] = 0

    while not rospy.is_shutdown():
        if not cam.get_frame():
            continue

        marker_dic_list, _ = avm.marker_recognition(cam.color_image)

        if marker_dic_list:
            send_tf(marker_dic_list)

            # keep track of all the kalman filter results
            for tid in markers:
                kalman_count[tid] += 1
                if kalman_count[tid] > 2000:
                    avm.remove_tid(tid)
                    kalman_count[tid] = 0

        # visualization
        overlay = cam.color_image.copy()
        overlay = overlay + avm.dimg
        avm.draw_markers(overlay)

        # publish debug image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', overlay)[1]).tostring()
        image_pub.publish(msg)
