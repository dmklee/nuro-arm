import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml
import time

from camera.utils import *
#TODO:
#   - handle error on connection, default to attempt reconnecting
    # toggle between camera_ids

class Camera:
    def __init__(self,
                 camera_id=1,
                 ):
        self._cam_id = camera_id

        self._cam = self._connect()

        self._calc_distortion_matrix()

        self._configs = self._read_configs()
        for k,v in self._configs.items():
            if isinstance(v, list):
                self._configs[k] = np.array(v)

        self._get_ego_pose()

    def set_camera_id(self, camera_id):
        self._cam_id = camera_id
        self._cam = self._connect()

    def _connect(self):
        return cv2.VideoCapture(self._cam_id)

    def _raw_image(self):
        ret, image = self._cam.read()
        if not ret:
            print('error, try reconnecting')
            return
        return image

    def get_image(self):
        img = self._raw_image()

        return self._undistort(img)


    def _disconnect(self):
        self._cam.release()

    def _calc_distortion_matrix(self):
        mtx, newcameramtx, roi, dist = calc_distortion_matrix()
        self._write_configs({
            'mtx' : mtx.tolist(),
            'undistort_mtx' : newcameramtx.tolist(),
            'undistort_roi' : roi,
            'dist_coeffs' : dist.tolist(),
        })

    def _get_ego_pose(self):
        if 'cam2world_rmat' in self._configs:
            return

        print('Calculating pose of camera...')
        # vec that goes from world origin to where right suction cup touches grid
        grid_tvec = np.array((70-6.5,77-6.5+20,0))
        gh, gw = (7,9)
        gsize = 20 # mm

        img = self.get_image()
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        ret, corners = cv2.findChessboardCorners(gray,
                                                (gh,gw),
                                                None)


        if ret:
            objp = np.zeros((gh*gw,3), np.float32)
            objp[:,:2] = gsize * np.dstack(np.mgrid[1:-gw+1:-1,gh:0:-1]).reshape(-1,2)
            objp += grid_tvec

            mtx = self._configs['undistort_mtx']
            dist = self._configs['dist_coeffs']

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)

            new_configs = {
                'cam2world_tvec' : tvecs[:,0],
                'cam2world_rmat' : cv2.Rodrigues(rvecs)[0],
                'world2cam_tvec' : -tvecs[:,0],
                'world2cam_rmat' : cv2.Rodrigues(-rvecs)[0]
            }
            self._configs.update(new_configs)
            self._write_configs({k:v.tolist() for k,v in new_configs.items()})
            return

        print('ERROR: checkerboard pattern was not identified.')
            # imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            # fig = plt.figure()
            # ax = plt.axes(projection='3d')
            # origin = np.zeros(3)
            # axis = 20*np.array(((1,0,0),(0,1,0),(0,0,1)))
            # axis_colors = 'gbr'
            # for a, c in zip(axis, axis_colors):
                # x,y,z = zip(origin, a)
                # ax.plot(x,y,z,'-', color=c)

            # cam_rot_mat, jcb = cv2.Rodrigues(-rvecs)
            # for a, c in zip(axis, axis_colors):
                # rot_a = np.dot(cam_rot_mat, a)
                # rot_vec = np.dot(cam_rot_mat, tvecs[:,0])
                # x,y,z = zip(origin - rot_vec, rot_a - rot_vec)
                # ax.plot(x,y,z,':', color=c)

            # for a, c in zip(axis, axis_colors):
                # rot_a = np.dot(world2grid_RMAT, a)
                # x,y,z = zip(world2grid_TVEC, rot_a+world2grid_TVEC)
                # ax.plot(x,y,z,'-', color=c)

            # ax.plot(*objp.T, 'k.')

            # plt.show()

            # img = draw(img, corners2, imgpts)
            # cv2.drawChessboardCorners(img, (7,9), corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(5000)
        # cv2.destroyAllWindows()

    def _undistort(self, img):
        # undistort
        dst = cv2.undistort(img,
                            self._configs['mtx'],
                            self._configs['dist_coeffs'],
                            None,
                            self._configs['undistort_mtx']
                           )
        # crop the image
        x, y, w, h = self._configs['undistort_roi']
        cropped = dst[y:y+h, x:x+w]
        return cropped

    def _write_configs(self, new_configs):
        doc = self._read_configs()

        doc.update(new_configs)
        with open("camera/configs.yaml", "w") as f:
            yaml.safe_dump(doc, f, default_flow_style=False)

        return doc

    def _read_configs(self):
        with open("camera/configs.yaml") as f:
            doc = yaml.safe_load(f)

        if doc is None:
            doc = dict()

        return doc

    def __call__(self):
        return self.get_image()

    def __del__(self):
        self._disconnect()

    def live_feed(self, frate=30):
        cv2.namedWindow("Live Feed")
        while True:
            img = self.get_image()
            cv2.imshow("Live Feed", img)
            cv2.waitKey(int(1000/frate))

        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = Camera(2)

    # calc_distortion_matrix()
    # camera.calc_camera_pse()
    # camera.live_feed()

