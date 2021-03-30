import numpy as np
import cv2
import os

def get_cube_poses():
    pass

def detect_blobs(color_thresholds):
    pass


def calc_distortion_matrix():
    folder = "camera/checkerboard_calibration/"
    positions = np.loadtxt(os.path.join(folder, "3dpositions.txt"),
                            delimiter=',')
    num_images = len(positions)

    GW, GH = 7, 9
    grid_size = 1 #mm
    objp = np.zeros((GW*GH,3), np.float32)
    objp[:,:2] = grid_size * np.mgrid[0:GW,0:GH].T.reshape(-1,2)

    img_pts = []
    obj_pts = []
    for img_id in range(num_images):
        img = cv2.imread(os.path.join(folder,f"{img_id}.jpg"))
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (GW,GH), None)

        if ret:
            img_pts.append(corners)
            obj_pts.append(objp)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts,
                                                       img_pts,
                                                       gray.shape[::-1],
                                                       None,
                                                       None)

    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist,
                                                      (w,h), 1,
                                                      (640, 480))

    return mtx, newcameramtx, roi, dist
