import cv2
from basecamera import camera
import os

class localvideo(camera):
    def __init__(self, device_id = 0):
        camera.__init__(self)
        self.videocap = cv2.VideoCapture('small.mp4')

    def get_frame(self):
        try:
            while (self.videocap.isOpened()):
                ret, frame = self.videocap.read()
                if not ret:
                    print "Video could not be read"
                    return

                # check if the camera read is True and the frame is not None
                if ret is True and frame is not None:
                    self.color_image = frame
                    cv2.imwrite(os.path.join(".", "mwc_video_test.jpg"), self.color_image)  # save frame as JPEG file
                    print("images are extacted")
                    return True
                else:
                    return False
        except Exception as e:
            print e
            print("web camera: failed to get color frame")
            return False

    def destroy(self):
                if not self.videocap:
                    # Release the video capture and close the window
                    self.videocap.release()
