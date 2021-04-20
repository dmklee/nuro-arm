import numpy as np
import cv2
import os

import neu_ro_arm.constants as constants

def find_arucotags(img, cam_mtx, dist_coeffs):
    gray = convert_gray(img)
    corners, ids, rejected = cv2.aruco.detectMarkers(
                                gray,
                                constants.aruco_dict,
                                parameters=constants.aruco_params,
                            )
    if ids is None:
        return []

    tag_size = constants.tag_size
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size,
                                                          cam_mtx, dist_coeffs)
    tags = []
    for i in range(len(corners)):
        cube2cam = transformation_matrix(rvecs[i], tvecs[i])
        tags.append({'tag_id' : ids[i][0],
                     'corners' : corners[i].reshape((4,2)),
                     'cube2cam' : cube2cam,
                     # 'rvec' : rvecs[i],
                     # 'tvec' : tvecs[i],
                    })

    return tags

def find_cubes(img, cam_mtx, dist_coeffs, cam2world):
    tags = find_arucotags(img, cam_mtx, dist_coeffs)

    cubes = []
    vertices = constants.cube_vertices.copy()
    vertices[:,2] -= 0.5*constants.cube_size

    for tag in tags:
        cube2world = np.dot(cam2world, tag['cube2cam'])
        tmp_vertices = coord_transform(cube2world, vertices)
        cube = {'pos' : cube2world[:3,3],
                'rot' : rotmat2euler(cube2world[:3,:3]),
                'id' : tag['tag_id'],
                'vertices' : tmp_vertices,
                'center' : tmp_vertices.mean(axis=0),
               }
        cubes.append(cube)

    return cubes

def project_world2pixel(pts_wframe, world2cam,
                       rvec, tvec, mtx, dist_coeffs):
    # transform world to camera frame
    pts_cframe = coord_transform(world2cam,
                                 pts_wframe)
    # project from camera frame to 2d pixel space
    pixels, _ = cv2.projectPoints( pts_wframe,
                                    rvec,
                                    tvec,
                                    mtx,
                                    dist_coeffs
                                   )
    return pixels[:,0,:]

def rvec2euler(rvec):
    return rotmat2euler(cv2.Rodrigues(rvec)[0])

def rvec2quat(rvec):
    euler = rvec2euler(rvec)
    return euler2quat(euler)

def euler2quat(euler):
    cy = np.cos(0.5 * euler[0])
    sy = np.sin(0.5 * euler[0])
    cp = np.cos(0.5 * euler[1])
    sp = np.sin(0.5 * euler[1])
    cr = np.cos(0.5 * euler[2])
    sr = np.sin(0.5 * euler[2])

    return np.array((cr*cp*cy+sr*sp*sy,
                     sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy))

def rotmat2euler(R):
    sy = np.sqrt(R[0,0]*R[0,0]+R[1,1]*R[1,1])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arxtan2(-R[1,2],R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array((x,y,z))

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

def transformation_matrix(rvec, tvec):
    mat = np.zeros((4,4))
    mat[:3,3] = tvec.flatten()
    mat[:3,:3] = cv2.Rodrigues(rvec)[0]
    mat[3,3] = 1
    return mat

def inverse_transformation_matrix(rvec, tvec):
    mat = np.zeros((4,4))
    rot_mat = cv2.Rodrigues(rvec)[0]
    # rotational matrices are orthogonal so inv <--> transpose
    inv_rot_mat = rot_mat.T
    mat[:3,3] = -np.dot(inv_rot_mat, tvec[:,0])
    mat[:3,:3] = inv_rot_mat
    mat[3,3] = 1
    return mat

def coord_transform(trans_mtx, pts):
    if len(pts.shape) == 1:
        pts = pts[None,:]
    homog_pts = np.concatenate( (pts, np.ones((len(pts),1))), axis=1 )
    new_homog_pts = np.dot(trans_mtx, homog_pts.T).T
    new_pts = np.true_divide(new_homog_pts[:,:-1],  new_homog_pts[:,[-1]])
    return new_pts

def calc_distortion_matrix(imgs=None, verbose=True):
    GW, GH = constants.calibration_gridshape
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
            img_pts2, _ = cv2.projectPoints(obj_pts[i], rvecs[1], tvecs[i], mtx, dist)
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
