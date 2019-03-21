import cv2
from basecamera import camera

#TODO: perform camera calibration and save and load the parameters from a file
camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)

class web_camera(camera):
    def __init__(self, device_id = 0):
        camera.__init__(self)
        self.cap = cv2.VideoCapture(device_id)

    def get_frame(self):
        try:
            ret, frame = self.cap.read()

            # check if the camera read is True and the frame is not None
            if ret is True and frame is not None:
                self.color_image = frame
                return True
            else:
                return False
        except Exception as e:
            print e
            print("web camera: failed to get color frame")
            return False

    def get_calibration(self):
        self.calibration_params = camera_params
        return self.calibration_params

    def destroy(self):
                if not self.cap:
                    # Release the video capture and close the window
                    self.cap.release()
                    cv2.destroyAllWindows()