import numpy as np
import cv2
from collections import namedtuple

import nuro_arm.constants as constants
import nuro_arm.transformation_utils as tfm

ArucoTag = namedtuple('ArucoTag', ['id_', 'corners', 'tag2cam'])
ArucoCube = namedtuple('ArucoCube', ['id_', 'pos', 'euler', 'vertices'])

# face_detector = cv2.CascadeClassifier('nuro_arm/camera/haarcascade_frontalface_default.xml')


def find_face(img):
    '''Detects location of face in image

    Parameters
    ----------
    img : array_like
        rgb or grayscale image

    Returns
    -------
    x : int
        horizontal position of face in pixel space
    y : int
        vertical position of face in pixel space
    w : int
        horizontal size of face in pixel space
    h : int
        vertical size of face in pixel space
    '''
    gray = convert_gray(img)
    faces, _, confidences = face_detector.detectMultiScale3(gray,
                                           scaleFactor=1.1,
                                           minNeighbors=7,
                                           minSize=(50,50),
                                           outputRejectLevels=True
                                          )
    if len(faces) == 0:
        return None

    #get max level_weights
    max_id = np.argmax(np.array(confidences).flatten())
    x,y,w,h = faces[max_id]
    return int(x+w/2), int(y+h/2), w, h

def find_arucotags(img, cam_mtx, dist_coeffs, tag_size=None):
    '''Locates aruco tags in image, recording info on ID, corner pixels, and tag pose

    Parameters
    ----------
    img: ndarray
        image taken by camera, will be converted to gray within the method
    cam_mtx: ndarray
        camera matrix; 2D array of shape (3,3); dtype=float
    dist_coeffs: ndarray
        distortion coefficients; array of length 5; dtype=float

    Returns
    -------
    list of namedtuple
        ArucoTags with fields: id_ (int), corners (ndarray; shape=(4,2);
        dtype=float), tag2cam (ndarray; shape=(4,4); dtype=float
    '''
    #TODO: alert if multiple of the same tag ids are detected as this may mess
    # up further processing
    gray = convert_gray(img)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray,
                                                     constants.ARUCO_DICT,
                                                     parameters=constants.ARUCO_PARAMS,
                                                    )
    if ids is None:
        return []

    if tag_size is None:
        tag_size = constants.TAG_SIZE
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size,
                                                          cam_mtx, dist_coeffs)
    tags = []
    for i in range(len(corners)):
        new_tag = ArucoTag(id_=ids[i][0],
                           corners=corners[i].reshape(4,2),
                           tag2cam=tfm.transformation_matrix(rvecs[i], tvecs[i])
                          )
        tags.append(new_tag)

    return tags

def find_cubes(img, cam_mtx, dist_coeffs, cam2world, cube_size=None, tag_size=None):
    '''Locates cubes in image by detecting aruco tags

    Because the tag consists of 4 coplanar points, the P2P calculation to determine
    the pose is subject to error.  This manifests in an odd "flipping" behavior
    as documented (here)[https://github.com/opencv/opencv/issues/8813]. Further
    processing should be used to filter out the anamolies or use some bias
    based on the fact that the cube will be lying flat.

    Given one of the poses, you can calculate the other possible P2P solution
    by mirroring the tag2cam rotation matrix in the z-direction (i.e. last row
    of the rotation matrix is negated)

    Parameters
    ----------
    img: ndarray
        image taken by camera, will be converted to gray within the method
    cam_mtx: ndarray
        camera matrix; 2D array of shape (3,3); dtype=float
    dist_coeffs: ndarray
        distortion coefficients; array of length 5; dtype=float
    cam2world: ndarray
        transformation matrix that converts from coordinate frame of the camera
        to the coordinate frame of the camera; shape=(4,2); dtype=float

    Returns
    -------
    namedtuple
        ArucoCubes with fields: id_ (int), pos (ndarray;shape=(3,);dtype=float),
        euler (ndarray; shape=(3,);dtype=float), vertices (ndarray; shape=(8,3);
        dtype=float.
    '''
    tags = find_arucotags(img, cam_mtx, dist_coeffs, tag_size)

    cubes = []

    for tag in tags:
        cubes.append(convert_tag_to_cube(tag, cam2world, cube_size))

    return cubes

def convert_tag_to_cube(tag, cam2world, cube_size=None):
    '''Uses coordinate transform to calculate cube properties from detected tag

    Parameters
    ----------
    tag : namedtuple
        ArucoTag
    cam2world : ndarray
        transformation matrix to convert from camera frame to world frame;
        shape=(4,4); dtype=float

    Returns
    -------
    namedtuple
        ArucoCube
    '''
    shift_center_mat = np.array(((1,0,0,0),
                                 (0,1,0,0),
                                 (0,0,1,-0.5*constants.CUBE_SIZE),
                                 (0,0,0,1)))
    cube2cam = np.dot(tag.tag2cam, shift_center_mat)
    cube2world = np.dot(cam2world, cube2cam)

    if cube_size is None:
        cube_size = constants.CUBE_SIZE
    cube_vertices = tfm.coord_transform(cube2world,
                                        cube_size * constants.NORM_CUBE_VERTICES)
    cube = ArucoCube(id_=tag.id_,
                pos=cube2world[:3,3],
                euler=tfm.rotmat2euler(cube2world[:3,:3]),
                vertices=cube_vertices,
               )

    return cube

def rotmat_median(rotmats):
    '''Return approximate geometric median of rotation matrices in O(N^2)

    This does not return the true geometric median, it returns the matrix
    that has the lowest sum of distances to other matrices

    Parameters
    ----------
    rotmats : ndarray
        array of rotation matrices; shape=(*,3,3); dtype=float

    Returns
    -------
    int :
        index of the median
    ndarray :
        median rotation matrix; shape=(3,3); dtype=float
    '''
    def dist_metric(R1, R2):
        '''Distance between rotation matrices as suggested
        (here)[https://github.com/opencv/opencv/issues/8813#issuecomment-583379900]
        '''
        mult = np.einsum('lij,ljk->lik', R1, np.swapaxes(R2, 1, 2) )
        return np.linalg.norm(np.log(mult), axis=(1,2))

    n_pts = len(rotmats)
    i,j = np.mgrid[:n_pts,:n_pts]
    R1 = rotmats[i.flatten()]
    R2 = rotmats[j.flatten()]

    distances = dist_metric(R1, R2).reshape(n_pts, n_pts)

    median_id = np.argmin(np.sum(distances, axis=0))

    return median_id, rotmats[median_id]

def project_to_pixels(pts, rvec, tvec, cam_mtx, dist_coeffs):
    '''Projects 3D world points to pixel locations

    Parameters
    ----------
    pts : ndarray
        sequence of 3D arrays representing x,y,z location in world
        coordinate frame
    rvec : ndarray
    tvec : ndarray
    cam_mtx : ndarray
        camera matrix; shape=(3,3); dtype=float
    dist_coeffs : ndarray
        distortion coefficients; shape=(5,);dtype=float

    Returns
    -------
    ndarray
        sequence of 2D pixel indices; shape=(*,2); dtype=float
    '''
    # project from camera frame to 2d pixel space
    pixels, _ = cv2.projectPoints(pts,
                                  rvec,
                                  tvec,
                                  cam_mtx,
                                  dist_coeffs
                                  )

    return pixels.reshape(-1,2)

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

def calc_distortion_matrix(imgs=None, verbose=True):
    GW, GH = constants.CALIBRATION_GRIDSHAPE
    grid_size = constants.calibration_gridsize
    objp = np.zeros((GW*GH,3), np.float32)
    objp[:,:2] = grid_size * np.mgrid[0:GW,0:GH].T.reshape(-1,2)

    criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    img_pts = []
    obj_pts = []
    for img in imgs:
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (GW,GH), None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            img_pts.append(corners2)
            obj_pts.append(objp)


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

    if verbose:
        total_error = 0
        for i in range(len(obj_pts)):
            img_pts2, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(img_pts[i], img_pts2, cv2.NORM_L2)/len(img_pts2)
            total_error += error
        print("===============")
        print(f"mtx : {mtx}")
        print(f"newcameramtx : {newcameramtx}")
        print(f"roi : {roi}")
        print(f"dist : {dist}")
        print(f"Total Error: {total_error/len(obj_pts)}")
        print("===============")

    return mtx, newcameramtx, roi, dist
