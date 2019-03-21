import pyrealsense2 as rs
import numpy as np
from basecamera import camera
# import cv2

camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)

class realsense(camera):
            def __init__(self, device_id = 0):
                camera.__init__(self)
                # Create a pipeline
                self.pipeline = rs.pipeline()

                # Create a config and configure the pipeline to stream
                #  different resolutions of color and depth streams
                self.config = rs.config()
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

                # Start streaming
                self.profile = self.pipeline.start(self.config)

                # Getting the depth sensor's depth scale (see rs-align example for explanation)
                self.depth_sensor = self.profile.get_device().first_depth_sensor()
                self.depth_scale = self.depth_sensor.get_depth_scale()
                print ("Depth Scale is: ", self.depth_scale)

                # We will be removing the background of objects more than
                #  clipping_distance_in_meters meters away
                clipping_distance_in_meters = 0.5  # 1 meter
                self.clipping_distance = clipping_distance_in_meters / self.depth_scale

                # Create an align object
                # rs.align allows us to perform alignment of depth frames to others frames
                # The "align_to" is the stream type to which we plan to align depth frames.
                self.align = rs.align(rs.stream.color)


            def get_aligned_realsense_frames(self):

                    frames = self.pipeline.wait_for_frames()
                    # frames.get_depth_frame() is a 640x360 depth image

                    aligned_frames = self.align.process(frames)
                    aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
                    aligned_color_frame = aligned_frames.get_color_frame()
                    if not aligned_depth_frame or not aligned_color_frame:
                        return None

                    depth_image = np.asanyarray(aligned_depth_frame.get_data())
                    color_image = np.asanyarray(aligned_color_frame.get_data())
                    images = color_image, depth_image
                    return images

            def get_frame(self):
                try:
                    self.color_image, self.depth_image = self.get_aligned_realsense_frames()
                    return True
                except Exception as e:
                    print("realsense: failed to get color and depth frames ", e)
                    return False

            def get_calibration(self):
                self.calibration_params = camera_params
                return self.calibration_params



            def release(self):
                pass

            def destroy(self):
                if not self.pipeline:
                    self.pipeline.stop()