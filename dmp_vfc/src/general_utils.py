import cv2
import numpy as np
import matplotlib.pyplot as plt


def non_max_suppression_fast(boxes, overlapThresh):
    """

    :param boxes: the rectangle boxes obtained from object detection
    :param overlapThresh: ratio between multiple boxes, usually between 0.1 ~ 0.3
    :return: a box with four points in image frame
    """

    if len(boxes) == 0:
        return []
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
    # initialize the list of picked indexes
    pick = []
    # grab the coordinates of the bounding boxes
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)
    while len(idxs) > 0:
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)

        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])

        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
        overlap = (w * h) / area[idxs[:last]]

        idxs = np.delete(idxs, np.concatenate(([last],
                                               np.where(overlap > overlapThresh)[0])))

    return boxes[pick].astype("int")


def houghLinePDetector(img):
    """

    :param img: a RGB image
    :return: a image with drawn Houghline
    """

    hpThreshold = cv2.getTrackbarPos('hpThreshold', 'HoughLineTuner')
    minLineLength = cv2.getTrackbarPos('minLineLength', 'HoughLineTuner')
    maxLineGap = cv2.getTrackbarPos('maxLineGap', 'HoughLineTuner')

    lines = cv2.HoughLinesP(cv2.Canny(img, 100, 200), 1, np.pi / 180, hpThreshold, minLineLength, maxLineGap)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return img


def houghLineTunerWindow():
    def nothing():
        pass

    hpThreshold, minLineLength, maxLineGap = [200] * 3

    cv2.namedWindow('HoughLineTuner')
    cv2.createTrackbar('hpThreshold', 'HoughLineTuner', hpThreshold, 500, nothing)
    cv2.createTrackbar('minLineLength', 'HoughLineTuner', minLineLength, 500, nothing)
    cv2.createTrackbar('maxLineGap', 'HoughLineTuner', maxLineGap, 500, nothing)


def get_3d(x, y, depth,
           camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)):
    """

    :help: https://en.wikipedia.org/wiki/Pinhole_camera_model
    :param x: x in image frame
    :param y: y in image frame
    :param depth: depth at (x,y) from point cloud
    :return: 3x1 position in camera frame
    """

    fx, fy, cx, cy = camera_params
    pos_3d = [0] * 3
    pos_3d[0] = (x - cx) * depth / fx
    pos_3d[1] = (y - cy) * depth / fy
    pos_3d[2] = depth
    return pos_3d


def get_camera_intrinsics(camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)):
    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    return K

def draw_rect(img, tag_size, tag_pose, z_sign=1,
              camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)):
    """

    :param img: an image
    :param tag_size: tag size in world frame
    :param tag_pose: 4x4 tag pose in camera frame
    """

    edges = np.array([
        0, 1, 1, 2, 2, 3, 3, 0,
        0, 4, 1, 5, 2, 6, 3, 7,
        4, 5, 5, 6, 6, 7, 7, 4
    ]).reshape(-1, 2)

    opoints = np.array([
        -1, -1, 0,
        1, -1, 0,
        1, 1, 0,
        -1, 1, 0,
        -1, -1, -2 * z_sign,
        1, -1, -2 * z_sign,
        1, 1, -2 * z_sign,
        -1, 1, -2 * z_sign,
    ]).reshape(-1, 1, 3) * 0.5 * tag_size

    # Rodrigues / Projection
    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    rvec, _ = cv2.Rodrigues(tag_pose[:3, :3])
    tvec = tag_pose[:3, 3]
    dcoeffs = np.zeros(5)
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]
    for j, k in edges:
        cv2.line(img, ipoints[j], ipoints[k], (255, 0, 0), 1, 16)


def draw_pose(img, pose, color=(0, 255, 255),
              camera_params = (616.0821208687582, 617.0408772886465, 298.35240138442555, 238.91077106418385)):
    """

    :param img: an image
    :param pose: 4x4 pose in camera_frame
    """

    hole = np.float32([[0, 0, 0]]).reshape(-1, 3)
    # Rodrigues / Projection
    K = get_camera_intrinsics()
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]
    dcoeffs = np.zeros(5)
    po, _ = cv2.projectPoints(hole, rvec, tvec, K, dcoeffs)
    po = np.round(po).astype(int)
    points = po.reshape(-1, 2)
    cv2.circle(img, tuple(points[0]), 1, color, 2)


def draw_axis(img, end_effector_pose):
    """

    :param img: an image
    :param end_effector_pose: 4x4 end effector pose in camera frame
    """
    axis = np.float32([[0, 0, 0], [5, 0, 0], [0, 5, 0], [0, 0, 5]]).reshape(-1, 3)
    K = get_camera_intrinsics()
    rvec, _ = cv2.Rodrigues(end_effector_pose[:3, :3])
    tvec = end_effector_pose[:3, 3]
    dcoeffs = np.zeros(5)
    po, _ = cv2.projectPoints(axis, rvec, tvec, K, dcoeffs)
    pts = np.round(po).astype(int)
    pts = pts.reshape(-1, 2)

    cv2.line(img, tuple(pts[0]), tuple(pts[1]), (255, 0, 0), 2, 0)
    cv2.line(img, tuple(pts[0]), tuple(pts[2]), (0, 255, 0), 2, 0)
    cv2.line(img, tuple(pts[0]), tuple(pts[3]), (0, 0, 255), 2, 0)


def get_roi(hole_pose, pos):
    # Rodrigues / Projection
    corner = np.float32([pos]).reshape(-1, 3)
    K = get_camera_intrinsics()
    rvec, _ = cv2.Rodrigues(hole_pose[:3, :3])
    tvec = hole_pose[:3, 3]
    dcoeffs = np.zeros(5)
    po, _ = cv2.projectPoints(corner, rvec, tvec, K, dcoeffs)

    po = np.round(po).astype(int)
    return po.reshape(-1, 2)


def rotation(theta_x, theta_y, theta_z):
    def rot_x(theta):
        return np.array([[1, 0, 0, 0],
                         [0, np.cos(theta), -np.sin(theta), 0],
                         [0, np.sin(theta), np.cos(theta), 0],
                         [0, 0, 0, 1]])

    def rot_y(theta):
        return np.array([[np.cos(theta), 0, np.sin(theta), 0],
                         [0, 1, 0, 0],
                         [-np.sin(theta), 0, np.cos(theta), 0],
                         [0, 0, 0, 1]])

    def rot_z(theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                         [np.sin(theta), 0, np.cos(theta), 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    return rot_x(theta_x).dot(rot_y(theta_y)).dot(rot_z(theta_z))


def translation(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])


def evaluation_xy(lt, sample):
    """
    ../img/*: generate noise measurement image for Kalman filter.
    """
    fig, ax = plt.subplots()
    ax.set_prop_cycle(color=['blue', 'blue', 'yellow', 'yellow', 'green', 'green', 'red', 'red', 'cyan', 'cyan'])
    for i in [x * 0.1 for x in range(5, 10)]:
        num = -int(i * sample)
        nt = -int((i - 0.1) * sample)
        xx, yy = [x[0, 3] for x in lt[num:nt]], [x[1, 3] for x in list[num:next]]
        plt.plot(xx, yy, '.', markersize=5)
        plt.plot(np.median(xx, axis=0), np.median(yy, axis=0), '^', markersize=15)
    qr_to_camera = np.median(lt[-int(0.9 * sample):], axis=0)
    ax.plot(qr_to_camera[0, 3], qr_to_camera[1, 3], 'o', color='black', markersize=20, label='Estimated position')

    plt.xlabel('X position / cm')
    plt.ylabel('Y position / cm')
    plt.title('Localization error')
    ax.legend(loc='bottom right', shadow=True, fontsize='x-large')
    plt.show()


def evaluation_z(lt, sample):
    """
    ../img/*: generate noise measurement image for Kalman filter.
    """
    fig, ax = plt.subplots()
    ax.set_prop_cycle(color=['blue', 'blue', 'yellow', 'yellow', 'green', 'green', 'red', 'red', 'cyan', 'cyan'])
    for i in [x * 0.1 for x in range(5, 10)]:
        num = -int(i * sample)
        nt = -int((i - 0.1) * sample)
        zz = [x[2, 3] for x in lt[num:nt]]
        plt.plot(np.zeros_like(zz), zz, '.', markersize=5)
        plt.plot(0, np.median(zz, axis=0), '^', markersize=15)
    qr_to_camera = np.median(lt[-int(0.9 * sample):], axis=0)
    ax.plot(0, qr_to_camera[2, 3], 'o', color='black', markersize=20, label='Estimated position')

    plt.ylabel('Z position / cm')
    plt.title('Localization error z axis')
    ax.legend(loc='bottom right', shadow=True, fontsize='x-small')
    plt.show()


