import numpy as np
import cv2
import apriltag
import general_utils as gu

# camera parameters 480P
camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)

# camera parameters 720P
# camera_params = (941.247, 941.247, 627.957, 350.391)

class Kalman(object):
    """
	Kalman filter to predict the next pose (4x4 matrix)
	"""

    def __init__(self, size):
        self.size = size
        self.kalman = cv2.KalmanFilter(self.size, self.size)
        self.kalman.measurementMatrix = np.eye(self.size, dtype=np.float32)
        self.kalman.transitionMatrix = np.eye(self.size, dtype=np.float32)
        self.kalman.processNoiseCov = 0.2 * np.eye(self.size, dtype=np.float32)

    def predict_pose(self, raw_pose, verbose=False):
        self.kalman.correct(np.reshape(raw_pose[0:3, :], self.size).astype(np.float32))
        prediction = np.reshape(self.kalman.predict(), (3, 4))
        if verbose:
            print 'The predicted pose is :'
            print prediction
        return np.vstack([prediction, np.matrix([0, 0, 0, 1])])


class ApriltagVisionModule(object):

	def __init__(self, markers):

		self.options = apriltag.DetectorOptions(families='tag36h11',
												border=1,
												nthreads=4,
												quad_decimate=1.0,
												quad_blur=0.0,
												refine_edges=True,
												refine_decode=False,
												refine_pose=False,
												debug=False,
												quad_contours=True)
		self.detector = apriltag.Detector(self.options)
		self.detector = apriltag.Detector(self.options,
										  searchpath=apriltag._get_demo_searchpath())
		self.kalman_filters = {}
		self.transformation_ka = Kalman(12)
		self.g_camera_base = None
		self.marker_pose_dict = {}
		self.markers = markers
		self.dimg = None # apriltage detection overlay matrix
		self.kalman_count = {}
		for tid in self.markers:
			self.kalman_count[tid] = 0

	@staticmethod
	def calibrate_camera():
		return camera_params

	# @staticmethod
	def marker_recognition(self, color_image):
		gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
		detections, self.dimg = self.detector.detect(gray_image, return_image=True)
		self.dimg = self.dimg[:, :, None] // 3

		if len(detections) >= 0:
			for detection in detections:
				tid = detection.tag_id
				if tid not in self.markers:
					continue
				pose, _, _ = self.detector.detection_pose(detection, camera_params, self.markers[tid])

				# convert into meters
				pose[:3, -1] = pose[:3, -1]/100.0
				self.marker_pose_dict[tid] = pose
				# if tid in track_marker_list:
				self.track_marker(tid)
				self.kalman_count[tid] = 0

			for tid in self.markers:
				if (tid not in detections) and (tid in self.kalman_filters):
					self.kalman_count[tid] += 1
				if self.kalman_count[tid] > 40:
					self.remove_tid(tid)


		return self.marker_pose_dict, self.dimg

	def remove_tid(self, tid):
		if tid in self.marker_pose_dict:
			self.marker_pose_dict.pop(tid)
		if tid in self.kalman_filters:
			self.kalman_filters.pop(tid)
			self.kalman_count[tid] = 0

	def track_marker(self, marker_id):
		try:
			#initialize kalman_filter
			if marker_id not in self.kalman_filters:
				self.kalman_filters[marker_id] = Kalman(12)
				self.kalman_count[marker_id] = 0

			self.marker_pose_dict[marker_id] = self.kalman_filters[marker_id].predict_pose(self.marker_pose_dict[marker_id])
			return True
		except:
			print ("tracker failed, marker_id %d, originial pose is None" % marker_id)
			return False

	def draw_markers(self, overlay):
		for tid in self.marker_pose_dict:

			tmp = self.marker_pose_dict[tid].copy()
			tmp[:3, -1] = tmp[:3, -1] * 100.0
			gu.draw_rect(overlay, self.markers[tid], tmp)
		return

	def camera_base_calibration(self, T_finger_to_gripper, T_gripper_to_base, hand_marker_id = 2):
		try:
			self.g_camera_base = self.marker_pose_dict[hand_marker_id].dot(T_finger_to_gripper).dot(np.linalg.inv(T_gripper_to_base))
			return True
		except:
			print("camera base calculation failed, check T_gripper_to_base or pose tracking")
			return False


if __name__ == '__main__':
	ttt = gu.rotation(0, np.pi / 8 + np.pi / 2, -np.pi / 2)
	g_marker_left = np.array([[1, 0, 0, 0.082 * 100],
							  [0, 1, 0, 0 * 100],
							  [0, 0, 1, 0.015 * 100],
							  [0, 0, 0, 1]]).dot(ttt)


	# apriltag vision module initialization with marker lists
	markers = {2: 3.47,6:2.5, 0: 8.3}
	avm = ApriltagVisionModule(markers)

	# camera selection parameter
	# 0: video samples (TODO)
	# 1: librealsense
	# 2: webcam
	# 3: kinect2 through ROS (TODO)

	camera_selection = 1
	cam = None

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
		# kinect2 TODO
		pass

	# kalman_count = {}
	# for tid in markers:
	# 	kalman_count[tid] = 0

	while True:
		if not cam.get_frame():
			continue

		# g_cam_base, left_effector_pose = avm.marker_recognition_dmp(cam.color_image, track_marker_list=track_marker_list)
		g_cam_base, left_effector_pose = avm.marker_recognition(cam.color_image)

		# if left_effector_pose is not None:
		# 	print g_cam_base
		# 	print left_effector_pose

		# # keep track of all the kalman filter results
		# for tid in markers:
		# 	kalman_count[tid] += 1
		# 	if kalman_count[tid] > 200:
		# 		avm.remove_tid(tid)
		# 		kalman_count[tid] = 0

		# visualization
		overlay = cam.color_image.copy()
		overlay = overlay + avm.dimg
		avm.draw_markers(overlay)

		cv2.namedWindow('marker.png', cv2.WINDOW_NORMAL)
		cv2.resizeWindow('marker.png', 960, 960)
		cv2.imshow('marker.png', overlay)
		cv2.waitKey(30)
