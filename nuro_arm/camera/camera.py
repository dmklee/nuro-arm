import numpy as np
import cv2
import time
import os

import nuro_arm.transformation_utils as tfm
from nuro_arm.camera import camera_utils
from nuro_arm.camera.gui import GUI
from nuro_arm.camera.capturer import Capturer, SimCapturer
import nuro_arm.constants as constants

class Camera:
    def __init__(self, camera_type='real', camera_id=None):
        '''Changes video capture to a different camera id number

        Parameters
        ----------
        camera_id: int, optional
            Camera number to read frames from.  The default value is None,
            meaning that the camera number will be taken from the config file.
            Use this parameter to override the config file

        Attributes
        ----------
        cap : obj
            Capturer instance used to read frames from camera
        gui : obj
            GUI instance used for plotting frames
        '''
        self.camera_type = camera_type
        assert camera_type in ('real', 'sim'), \
                'Invalid argument for camera_type'
        self.cap = Capturer() if camera_type == 'real' else SimCapturer()
        self.gui = GUI(self.cap)

        self.load_configs()

        # override camera_id config
        if camera_id is not None:
            self._camera_id = camera_id

        is_connected = self.connect()
        if not is_connected:
            print(f'[ERROR] Failed to connect to camera{camera_id}.')
            return

    def connect(self):
        '''Sets up connection to camera

        Returns
        -------
        bool
            True if connection was successful, False otherwise
        '''
        is_connected = self.cap.set_camera_id(self._camera_id)
        return is_connected

    def change_camera_id(self, cam_id):
        '''Change connection to another camera id.  If already connected to
        specified camera id, then nothing happens

        Parameters
        ----------
        cam_id : int
            camera id

        Returns
        -------
        bool
            True if connection was made, False if connection failed
        '''
        if cam_id == self._camera_id:
            return True
        self._camera_id = cam_id
        return self.cap.set_camera_id(self._camera_id)

    @property
    def frame_rate(self):
        '''Frame rate of the Capturer attribute

        Returns
        -------
        int
            Frames per second
        '''
        return self.cap._frame_rate

    def calc_location(self):
        '''Determine camera pose in world frame by localizing checkerboard pattern

        Returns
        -------
        bool
            True if location was determined, False otherwise.  Failure will occur
            if camera did not return frame or if checkerboard pattern could
            not be identified in the image.
        tuple
            rvec : ndarray
                see cv2.Rodrigues for info on rotation vector
            tvec : ndarray
                translation vector
            world2cam : ndarray
                4x4 transformation matrix that transforms homogeneous vector from
                world coordinate frame to camera coordinate frame
            cam2world : ndarray
                4x4 transformation matrix that transforms homogeneous vector from
                camera coordinate frame to world coordinate frame
        '''
        if self.camera_type == 'sim':
            print('WARNING: Calibration is not supported for simulated camera. '
                  'Simulated cameras location must be set by user.')
            return False, None

        gh, gw = constants.CALIBRATION_GRIDSHAPE
        gsize = constants.CALIBRATION_GRIDSIZE

        img = self.get_image()
        gray = camera_utils.convert_gray(img)

        ret, corners = cv2.findChessboardCorners(gray, (gh, gw), None)

        if ret:
            # coordinates of grid corners in world coordinates
            objp = np.zeros((gh*gw, 3), np.float32)
            objp[:, :2] = gsize * np.dstack(np.mgrid[1:gw+1, -1:gh-1]).reshape(-1, 2)
            objp += constants.TVEC_WORLD2RIGHTFOOT

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)

            ret, rvec, tvec = cv2.solvePnP(objp, corners2, self._mtx, self._dist_coeffs)

            world2cam = tfm.transformation_matrix(rvec, tvec)
            cam2world = tfm.inverse_transformation_matrix(rvec, tvec)

            return True, (rvec, tvec, world2cam, cam2world)

        return False, None

    def set_location(self, pose_mtx):
        '''Set camera pose in world frame, only valid for simulated camera

        Parameters
        ----------
        pose_mtx : array_like
            4x4 pose matrix of camera in world frame
        '''
        if self.camera_type == 'real':
            print("WARNING: Real camera's pose cannot be set by user.")
            return

        self._cap.view_mtx = np.array(pose_mtx).reshape(-1)
        #TODO: reassign self._rvec, self._tvec

    def start_recording(self, duration):
        '''Starts recording on camera

        Parameters
        ----------
        duration : float
            Seconds to record camera feed
        '''
        self.cap.start_recording(duration)

    def end_recording(self):
        '''Ends recording on camera
        '''
        self.cap.end_recording()

    def wait_for_recording(self):
        '''Waits until camera is done recording, then returns recorded frames

        Returns
        -------
        ndarray
            sequence of images
        '''
        while self.cap.is_recording():
            time.sleep(0.1)
        return self.cap.get_recording()

    def project_world_points(self, pts_wframe):
        '''Projects 3D world points to pixel locations in camera feed

        Parameters
        ----------
        pts_wframe : ndarray
            sequence of 3D arrays representing x,y,z location in world
            coordinate frame

        Returns
        -------
        ndarray
            sequence of 2D pixel indices; shape=(*,2); dtype=float
        '''
        return camera_utils.project_to_pixels(pts_wframe,
                                              self._rvec,
                                              self._tvec,
                                              self._mtx,
                                              self._dist_coeffs)

    def load_configs(self):
        '''Reads config file writing to private attributes

        Returns
        -------
        bool
            True if configs were loaded succesfully, False otherwise
        '''
        self._mtx = constants.CAM_MTX
        self._dist_coeffs = constants.CAM_DIST_COEFFS
        self._camera_id = 0

        if os.path.exists(constants.CAMERA_CONFIG_FILE):
            data = np.load(constants.CAMERA_CONFIG_FILE, allow_pickle=True).item()
            try:
                self._camera_id = data.get('camera_id')
                self._rvec = data.get('rvec')
                self._tvec = data.get('tvec')
                self._world2cam = data.get('world2cam')
                self._cam2world = data.get('cam2world')
                return True
            except IndexError:
                pass

        print('[WARNING] Camera config file not found. '
              ' Calibration should be performed.')
        return False

    @property
    def configs(self):
        return {'camera_id': self._camera_id,
                'rvec': self._rvec,
                'tvec': self._tvec,
                'world2cam': self._world2cam,
                'cam2world': self._cam2world,
                'mtx': self._mtx,
                'dist_coeffs': self._dist_coeffs,
                }

    def get_image(self):
        '''Get frame from capturer

        Returns
        -------
        img : ndarray
        '''
        img = self.cap.read()
        return img

    def __call__(self):
        return self.get_image()
