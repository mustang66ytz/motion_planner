class camera(object):
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.calibration_params = None

    def get_frame(self):
        pass

    def get_calibration(self):
        pass

    #TODO perform 2D camera calibration for camera intrinsics
    def calibration(self):
		pass

