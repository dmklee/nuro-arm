import numpy as np
import cv2
import os

def reshape_image(img, height, width=None):
    if width is None:
        width = height
    reshaped = cv2.resize(img, (width, height))
    return reshaped

def rescale_image(img, scale):
    new_height = int(img.shape[0] * scale)
    new_width = int(img.shape[1] * scale)
    return reshape_image(img, new_height, new_width)

def convert_gray(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def get_cube_poses():
    pass

def calc_distortion_matrix():
    folder = "camera/calibration_photos/photos/"

    GW, GH = 7, 9
    grid_size = 20 #mm
    objp = np.zeros((GW*GH,3), np.float32)
    objp[:,:2] = grid_size * np.mgrid[0:GW,0:GH].T.reshape(-1,2)

    img_pts = []
    obj_pts = []
    img_id = 0
    while True:
        img = cv2.imread(os.path.join(folder,f"{img_id}.png"))
        if img is None:
            break
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        img_id += 1

        ret, corners = cv2.findChessboardCorners(gray, (GW,GH), None)

        if ret:
            img_pts.append(corners)
            obj_pts.append(objp)
            # cv2.drawChessboardCorners(img, (GW,GH), corners, ret)
            # cv2.imshow('img', img)
            # cv2.waitKey(100)
    # num_images = img_id
    # cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts,
                                                       img_pts,
                                                       gray.shape[::-1],
                                                       None,
                                                       None)

    h,  w = gray.shape
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,
                                                      dist,
                                                      (w,h),
                                                      1,
                                                      (w,h)
                                                     )

    # for img_id in range(num_images):
        # img = cv2.imread(os.path.join(folder,f"{img_id}.png"))
        # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        # # crop the image
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        # cv2.drawChessboardCorners(img, (GW,GH), corners, True)
        # cv2.imshow('img', dst)
        # cv2.waitKey(500)
    # cv2.destroyAllWindows()


    return mtx, newcameramtx, roi, dist
