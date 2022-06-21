import numpy as np
from scipy.spatial.transform import Rotation as R

#ToDo: add docstrings

def transformation_matrix(rvec, tvec):
    '''rvec and tvec both (3,1)'''
    mtx = np.eye(4, dtype=float)
    mtx[:3,3] = tvec.flatten()
    mtx[:3,:3] = R.from_rotvec(rvec.flatten()).as_matrix()
    return mtx

def unpack_rvec_tvec(mtx):
    rvec = R.from_matrix(mtx[:3,:3]).as_rotvec()
    tvec = mtx[:3,3]
    return rvec, tvec

def invert_transformation_matrix(mtx):
    inverted = np.copy(mtx)
    inverted[:3,:3] = mtx[:3,:3].T
    inverted[:3,3] = -np.dot(inverted[:3,:3], mtx[:3,3])
    return inverted

def rotmat_to_quaternion(rotmat):
    return R.from_matrix(rotmat).as_quat()

def apply_transformation(transform_mtx, pts):
    if len(pts.shape) == 1:
        pts = pts[None,:]
    homog_pts = np.concatenate( (pts, np.ones((len(pts),1))), axis=1 )
    new_homog_pts = np.dot(transform_mtx, homog_pts.T).T
    new_pts = np.true_divide(new_homog_pts[:,:-1],  new_homog_pts[:,[-1]])
    return new_pts
