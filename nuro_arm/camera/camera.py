import numpy as np
import pybullet as pb
import cv2
import time
import os

import nuro_arm.transformation_utils as tfm
from nuro_arm.camera import camera_utils
from nuro_arm.camera.gui import GUI
from nuro_arm.camera.capturer import Capturer, SimCapturer
from nuro_arm import constants

class Camera:
    def __init__(self,
                 camera_type='real',
                 camera_id=None,
                 pose_mtx=None,
                 free_floating=True,
                 run_async=True,
                 pb_client=0):
        '''Camera class that controls real or simulated camera feed, and is
        calibrated to allow for conversion from pixel to cartesian space

        Parameters
        ----------
        camera_type : str, default to 'real'
            Whether to connect to a image feed from a real or simulated camera.
            Must be either 'real' or 'sim'
        camera_id : int, optional
            Camera number to read frames from.  The default value is None,
            meaning that the camera number will be taken from the config file.
            Use this parameter to override the config file.  This is **not**
            used for simulated camera
        pose_mtx : array_like, optional
            4x4 Transformation matrix describing pose of camera in base frame.
            Only used for simulated camera; for real camera, pose is calculated
            during calibration.  If not provided, then the default camera
            pose from `nuro_arm.constants` is used.
        free_floating : bool, default to False
            If True, then the collision objects of the camera and camera stand
            are not added to simulator.  This can be used for a real or simulated
            camera, and will impact collision detection by the robot.  Only set to
            False if the camera is well out of range of arm.
        pb_client : int, optional
            Physics client id number in which the camera will be placed.  This is
            used even for real camera, where it controls which simulator the
            collision objects are added to.

        Attributes
        ----------
        cap : obj
            Capturer instance used to read frames from camera
        gui : obj
            GUI instance used for plotting frames
        '''
        self._camera_id = 0
        self._pb_client = pb_client

        self.free_floating = free_floating
        self.camera_collision_obj = None
        self.rod_collision_obj = None

        self.camera_type = camera_type
        assert camera_type in ('real', 'sim'), 'Invalid argument for camera_type'
        if camera_type == 'real':
            self.cap = Capturer()
            self.load_configs()
            # override camera_id config
            if camera_id is not None:
                self._camera_id = camera_id
            self.add_collision_objects()
        else:
            self.cap = SimCapturer()
            if pose_mtx is None:
                pose_mtx = constants.DEFAULT_CAM_POSE_MTX
            self.set_location(pose_mtx)

        self.gui = GUI(self.cap)

        if not self.connect(run_async):
            print(f'[ERROR] Failed to connect to camera{camera_id}.')


    def connect(self, run_async):
        '''Sets up connection to camera

        Returns
        -------
        bool
            True if connection was successful, False otherwise
        '''
        is_connected = self.cap.set_camera_id(self._camera_id, run_async)
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
            print('[WARNING:] Calibration is not supported for simulated camera. '
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

            ret, rvec, tvec = cv2.solvePnP(objp, corners2,
                                           constants.NEW_CAM_MTX,
                                           np.zeros(5))

            world2cam = tfm.transformation_matrix(rvec, tvec)
            cam2world = tfm.invert_transformation_matrix(world2cam)

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
            print("[WARNING:] Real camera's pose cannot be set by user.")
            return

        self.cap.set_pose_mtx(pose_mtx)
        self._cam2world = pose_mtx
        self._world2cam = tfm.invert_transformation_matrix(pose_mtx)
        self._rvec, self._tvec = tfm.unpack_rvec_tvec(self._world2cam)

        # update collision object location
        self.add_collision_objects()

    def add_collision_objects(self):
        if self.free_floating:
            return

        self.remove_collision_objects()

        cam_pos, cam_quat, rod_pos, rod_quat = self._unpack_camera_pose(self._cam2world)

        camera_urdf_path = os.path.join(constants.URDF_DIR, 'camera.urdf')
        rod_urdf_path = os.path.join(constants.URDF_DIR, 'camera_rod.urdf')
        self.camera_collision_obj = pb.loadURDF(camera_urdf_path, cam_pos, cam_quat,
                                                physicsClientId=self._pb_client)
        self.rod_collision_obj = pb.loadURDF(rod_urdf_path, rod_pos, rod_quat,
                                             physicsClientId=self._pb_client)

    def remove_collision_objects(self):
        if self.camera_collision_obj is not None:
            pb.removeBody(self.camera_collision_obj,
                          physicsClientId=self._pb_client)
            self.camera_collision_obj = None
        if self.rod_collision_obj is not None:
            pb.removeBody(self.rod_collision_obj,
                          physicsClientId=self._pb_client)
            self.rod_collision_obj = None

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
        return camera_utils.project_to_pixels(pts_wframe, self._cam2world)

    def load_configs(self):
        '''Reads config file writing to private attributes

        Returns
        -------
        bool
            True if configs were loaded succesfully, False otherwise
        '''
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

    def get_image(self):
        '''Get frame from capturer

        Returns
        -------
        img : ndarray
        '''
        img = self.cap.read()
        return img

    @property
    def configs(self):
        return {'camera_id': self._camera_id,
                'rvec': self._rvec,
                'tvec': self._tvec,
                'world2cam': self._world2cam,
                'cam2world': self._cam2world,
                }

    def _unpack_camera_pose(self, pose_mtx):
        '''Get params for positioning camera and rod based on pose of camera

        Parameters
        ----------
        pose_mtx: ndarray
            Transformation matrix from world frame to camera frame; shape=(4,4);
            dtype=float

        Returns
        -------
        cam_pos : array_like
            3d postion vector of camera body
        cam_quat : array_like
            quaternion of camera body
        rod_pos : array_like
            3d postion vector of rod body
        rod_quat : array_like
            quaternion of rod body
        '''
        cam_pos = pose_mtx[:3,3]
        cam_rotmat = pose_mtx[:3,:3]
        cam_quat = tfm.rotmat_to_quaternion(cam_rotmat)

        rod_offset_vec = np.array((0.026, -0.012, -0.013))
        rod_pos = cam_pos + np.dot(cam_rotmat, rod_offset_vec)
        rod_pos[2] = 0
        rod_quat = (0,0,0,1)

        return cam_pos, cam_quat, rod_pos, rod_quat

    def find_cubes(self, cube_size=None, tag_size=None):
        '''Identify cubes in image, given that an Aruco tag is located in the
        center of one of its visible faces.

        Parameters
        ----------
        tag_size: float, optional
            Size of aruco tag in meters.  If not provided, the default value
            from `nuro_arm.constants` will be used.
        cube_size: float, optional
            Side length of cube in meters.  If not provided, the default value
            from `nuro_arm.constants` will be used.

        Returns
        -------
        list
            List of ArucoCube namedtuples, see `nuro_arm/camera/camera_utils.py`
        '''
        img = self.get_image()
        return camera_utils.find_cubes(img, self._cam2world, cube_size, tag_size)

    def get_pb_client(self):
        return self._pb_client

    def __call__(self):
        return self.get_image()
