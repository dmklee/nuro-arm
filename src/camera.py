import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml

from src.camera_utils import *
import src.constants as constants
#TODO:
#   - handle error on connection, default to attempt reconnecting
    # toggle between camera_ids

class Camera:
    CONFIG_FILE = "src/configs/camera.npz"
    def __init__(self,
                 camera_id=1,
                 ):
        self._cam_id = camera_id

        self._cam = self._connect()

        self._calc_distortion_matrix()
        asdf

        self._configs = self._read_configs()

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
        print('Calculating pose of camera...')
        # vec that goes from world origin to where right suction cup touches grid
        gh, gw = constants.calibration_gridshape
        gsize = constants.calibration_gridsize

        img = self.get_image()
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        ret, corners = cv2.findChessboardCorners(gray,
                                                (gh,gw),
                                                None)


        if ret:
            objp = np.zeros((gh*gw,3), np.float32)
            objp[:,:2] = gsize * np.dstack(np.mgrid[1:-gw+1:-1,gh:0:-1]).reshape(-1,2)
            objp += constants.tvec_world2rightfoot

            mtx = self._configs['undistort_mtx']
            dist = self._configs['dist_coeffs']

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)


            # axis =  np.float32([[20,0,0],[0,20,0],[0,0,20],[0,0,0]]).reshape(-1,3)
            # axis += constants.tvec_world2rightfoot
            # imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            # print(tuple(imgpts[-1].ravel()))
            # def draw(img, corners, imgpts):
                # img = cv2.line(img, tuple(imgpts[-1].ravel()), tuple(imgpts[0].ravel()), (255,0,0), 5)
                # img = cv2.line(img, tuple(imgpts[-1].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 5)
                # img = cv2.line(img, tuple(imgpts[-1].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 5)
                # return img
            # img = draw(img, corners2, imgpts)
            # cv2.imshow('img', img)
            # cv2.waitKey(3000)

            new_configs = {
                'rvec' : rvecs,
                'tvec' : tvecs,
                'world2cam' : transformation_matrix(rvecs, tvecs),
                'cam2world' : inverse_transformation_matrix(rvecs, tvecs),
            }
            self._configs = self._write_configs(new_configs)
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

            # # cam_rot_mat, jcb = cv2.Rodrigues(-rvecs)
            # cam_axis = coord_transform(self._configs['cam2world'],
                                       # axis)
            # cam_origin = coord_transform(self._configs['cam2world'],
                                         # origin)[0]
            # for a, c in zip(cam_axis, axis_colors):
                # x,y,z = zip(cam_origin, a)
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

    def project_world_points(self, pts_wframe):
        # transform world to camera frame
        pts_cframe = coord_transform(self._configs['world2cam'],
                                     pts_wframe)
        # project from camera frame to 2d pixel space
        img_pts, _ = cv2.projectPoints( pts_wframe,
                                        self._configs['rvec'],
                                        self._configs['tvec'],
                                        self._configs['undistort_mtx'],
                                        self._configs['dist_coeffs']
                                       )
        return img_pts

    def _write_configs(self, new_configs):
        configs = self._read_configs()

        configs.update(new_configs)
        np.savez(self.CONFIG_FILE, **configs)
        # with open("src/configs/camera.yaml", "w") as f:
            # yaml.safe_dump(doc, f, default_flow_style=False)

        return configs

    def _read_configs(self):
        # with open("src/configs/camera.yaml") as f:
            # doc = yaml.safe_load(f)
        configs = dict(np.load(self.CONFIG_FILE))

        if configs is None:
            configs = dict()

        return configs

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
    camera = Camera(0)
    pts = constants.tvec_world2rightfoot
    grid = 20*np.mgrid[-7:2,1:8].T.reshape(-1,2)+constants.tvec_world2rightfoot[:2]
    pts = np.zeros((len(grid),3))
    pts[:,:2] = grid
    pts = camera.project_world_points(pts)
    img = camera.get_image()
    plt.figure()
    plt.imshow(img)
    plt.plot(pts[...,0], pts[...,1], 'r.')
    plt.axis('off')
    plt.show()

    # camera.calc_camera_pse()
    # camera.live_feed()

