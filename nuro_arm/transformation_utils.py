import numpy as np
import cv2
from scipy.spatial.transform import Rotation

#TODO: substitute functionality from scipy where available

def rvec2euler(rvec):
    '''Converts rotation vector (Rodrigues) to euler angles

    Returns
    -------
    ndarray
        euler angles; shape=(3,); dtype=float
    '''
    return rotmat2euler(cv2.Rodrigues(rvec)[0])

def rvec2quat(rvec):
    '''Converts rotation vector (Rodrigues) to quaternion

    Returns
    -------
    ndarray
        quaternion; shape=(4,); dtype=float
    '''
    euler = rvec2euler(rvec)
    return euler2quat(euler)

def euler2quat(euler):
    '''Converts euler angles to quaternion

    Returns
    -------
    ndarray
        quaternion; shape=(4,); dtype=float
    '''
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
    '''Converts rotation matrix to euler angles

    Returns
    -------
    ndarray
        euler angles; shape=(3,); dtype=float
    '''
    sy = np.sqrt(R[0,0]*R[0,0]+R[1,1]*R[1,1])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2],R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array((x,y,z))

def transformation_matrix(rvec, tvec):
    mat = np.zeros((4,4))
    mat[:3,3] = tvec.flatten()
    mat[:3,:3] = cv2.Rodrigues(rvec)[0]
    mat[3,3] = 1
    return mat

def rotmat2rodrigues(rotmat):
    r = Rotation.from_matrix(rotmat)
    return r.as_rotvec().reshape(3,1)

def inverse_transformation_matrix(rvec, tvec):
    mat = np.zeros((4,4))
    rot_mat = cv2.Rodrigues(rvec)[0]
    # rotational matrices are orthogonal so inv <--> transpose
    inv_rot_mat = rot_mat.T
    mat[:3,3] = -np.dot(inv_rot_mat, tvec[:,0])
    mat[:3,:3] = inv_rot_mat
    mat[3,3] = 1
    return mat

def invert_transformation(mtx):
    inverted = np.copy(mtx)
    inverted[:3,:3] = mtx[:3,:3].T
    inverted[:3,3] = -np.dot(inverted[:3,:3], mtx[:3,3])
    return inverted

def coord_transform(transform_mtx, pts):
    if len(pts.shape) == 1:
        pts = pts[None,:]
    homog_pts = np.concatenate( (pts, np.ones((len(pts),1))), axis=1 )
    new_homog_pts = np.dot(transform_mtx, homog_pts.T).T
    new_pts = np.true_divide(new_homog_pts[:,:-1],  new_homog_pts[:,[-1]])
    return new_pts

