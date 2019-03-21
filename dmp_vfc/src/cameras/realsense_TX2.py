#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CompressedImage
from realsense2_cam.msg import ColorDepthImage
#import ColorDepthImage.msg

import numpy as np
from basecamera import camera
import cv2

camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)

class realsenseTX2(camera):
            def __init__(self, device_id = 0):
                camera.__init__(self)

                # rospy.init_node('realsenseTX2', anonymous=True)

                # Getting the depth sensor's depth scale (see rs-align example for explanation)
                self.depth_scale = 1
                print ("Depth Scale is: ", self.depth_scale)

                # We will be removing the background of objects more than
                #  clipping_distance_in_meters meters away
                clipping_distance_in_meters = 0.5  # 1 meter
                self.clipping_distance = clipping_distance_in_meters / self.depth_scale
                self.bridge = CvBridge()

                #rospy.Subscriber('/realsense2_cam/rgb_jpeg_stream', CompressedImage, self.callback_color_image)
                rospy.Subscriber('/realsense2_cam/rgb_depth_stream_raw', ColorDepthImage, self.callback_colordepth_image)
                # rospy.Subscriber('/realsense2_cam/rgb_depth_stream', ImageList, self.callback_depth_image)
                # rospy.Subscriber('/realsense2_cam/rgb_depth_stream_high_framerate', ImageList, self.callback_depth_image)

            def callback_depth_image(self, data):
                # parse data
                #print('depth frame...')
                self.depth_image = data.data
                self.rgb_image = data.data
                pass

            def callback_color_image(self, data):
                try:
                    print('color frame...')
                    np_arr = np.fromstring(data.data, np.uint8)
                    np_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    self.color_image = np_image
                except CvBridgeError, e:
                    print(e)

            def callback_colordepth_image(self, data):
                try:
                    # print('color depth frame...')
                    self.color_image = self.bridge.imgmsg_to_cv2(data.color, "bgr8")
                    self.depth_image = self.bridge.imgmsg_to_cv2(data.depth, "16UC1")

                    # cv2.imshow('color', self.color_image)
                    # cv2.imshow('depth', self.depth_image)
                    # cv2.waitKey(3)

                except CvBridgeError, e:
                    print(e)

            def get_aligned_realsense_frames(self):

                images = self.rgb_image, self.depth_image
                return images

            def get_frame(self):
                try:
                    if self.color_image is None:
                        return False
                    return True
                except Exception as e:
                    print("realsense: failed to get color and depth frames ", e)
                    return False

            def get_calibration(self):
                pass



            def release(self):
                pass

            def destroy(self):
                pass


if __name__ == '__main__':


    rospy.init_node('test')
    cam = realsenseTX2()

    rospy.spin()