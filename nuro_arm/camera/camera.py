import numpy as np
import cv2
import time


import neu_ro_arm.transformation_utils as tfm
from neu_ro_arm.camera import camera_utils
from neu_ro_arm.camera.gui import GUI
from neu_ro_arm.camera.capturer import Capturer
import neu_ro_arm.constants as constants

class Camera:
    CONFIG_FILE = "neu_ro_arm/camera/configs.npz"
    def __init__(self, camera_id=None):
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
        self.cap = Capturer()
        self.gui = GUI(self.cap)

        self.unpack_configs()

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

    @property
    def frame_rate(self):
        '''Frame rate of the Capturer attribute

        Returns
        -------
        int
            Frames per second
        '''
        return self.cap._frame_rate

    def _calc_distortion_matrix(self):
        mtx, newcameramtx, roi, dist = camera_utils.calc_distortion_matrix()
        self._update_config_file({
            'mtx' : mtx.tolist(),
            'undistort_mtx' : newcameramtx,
            'undistort_roi' : roi,
            'dist_coeffs' : dist,
        })

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
        gh, gw = constants.calibration_gridshape
        gsize = constants.calibration_gridsize

        img = self.get_image()
        gray = camera_utils.convert_gray(img)

        ret, corners = cv2.findChessboardCorners(gray,
                                                (gh,gw),
                                                None)

        if ret:
            # coordinates of grid corners in world coordinates
            objp = np.zeros((gh*gw,3), np.float32)
            objp[:,:2] = gsize * np.dstack(np.mgrid[1:-gw+1:-1,gh:0:-1]).reshape(-1,2)
            objp += constants.tvec_world2rightfoot

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)

            ret, rvec, tvec = cv2.solvePnP(objp, corners2, self._mtx, self._dist_coeffs)

            world2cam = tfm.transformation_matrix(rvec, tvec)
            cam2world = tfm.inverse_transformation_matrix(rvec, tvec)

            return True, (rvec, tvec, world2cam, cam2world)

        return False, None

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

    def undistort(self, img):
        '''Corrects for distortion in image

        Parameters
        ----------
        img : ndarray
            2D or 3D image

        Returns
        -------
        ndarray
            image that will be strictly smaller in height/width than input image
        '''
        dst = cv2.undistort(img,
                            self._mtx,
                            self._dist_coeffs,
                            None,
                            self._undistort_mtx
                           )
        # crop the image
        x, y, w, h = self._undistort_roi
        cropped = dst[y:y+h, x:x+w]
        return cropped

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
        return cam_utils.project_to_pixels(pts_wframe,
                                           self._rvec,
                                           self._tvec,
                                           self._mtx,
                                           self._dist_coeffs
                                          )

    def _update_config_file(self, new_configs):
        '''Adds new configs to config file, or updates those that already exist.

        Parameters
        ----------
        new_configs : dict
            Dictionary of configs.  Values should be ndarray type
        '''
        configs = np.load(self.CONFIG_FILE)
        configs = dict(configs) if configs is not None else dict()

        configs.update(new_configs)
        np.savez(self.CONFIG_FILE, **configs)

    def unpack_configs(self, write=True):
        '''Reads config file, writing to private attributes if desired

        Parameters
        ----------
        write : bool, default to True
            True if configs should be written to private attributes

        Raises
        ------
        KeyError
            If one of the private attributes does not exist in the config
            file.

        Returns
        -------
        dict
            configs that were found in config file
        '''
        configs = dict(np.load(self.CONFIG_FILE))

        if write:
            try:
                self._camera_id = int(configs['camera_id'])
                self._rvec = configs['rvec']
                self._tvec = configs['tvec']
                self._mtx = configs['mtx']
                self._undistort_mtx = configs['undistort_mtx']
                self._undistort_roi = configs['undistort_roi']
                self._dist_coeffs = configs['dist_coeffs']
                self._world2cam = configs['world2cam']
                self._cam2world = configs['cam2world']
            except KeyError as e:
                print(f'[ERROR] Some camera configs [{e}] are missing. Run setup_camera.py '
                      'to properly populate config file.')

        return configs

    def get_image(self):
        '''Get frame from capturer

        Returns
        -------
        img : ndarray
        '''
        img = self.cap.read()
        return img

    # def get_cubes(self, n_frames=10, min_frames=5):
        # '''Calculate position of cubes using aruco tags, position is refined
        # over multiple frames to improve accuracy
        # '''
        # positions = dict()
        # rotmats = dict()
        # for i in range(n_frames):
            # img = self.cap.read()
            # cubes = camera_utils.find_cubes(img)
            # for cube in cubes:
                # existing_positions = positions.get(cube.tag_id, [])
                # existing_rotmats = rotmats.get(cube.tag_id, [])
                # positions[cube.tag_id] = existing_positions + [cubes.pos]
                # rotmats[cube.tag_id] = existing_rotmats + [cubes.rotmat]

        # # process images
        # refined_cubes = {}
        # for tag_id in positions.keys():
            # these_rotmats = rotmats[tag_id]
            # if len(these_rotmats) < min_frames:
                # continue
            # median_id, _ = camera_utils.rotmat_median(these_rotmats)
            # refined_cubes[tag_id] = {'pos' : positions[tag_id][median_id],
                                     # 'rotmat' : rotmats[tag_id][median_id]
                                    # }
        # return refined_cubes

    def __call__(self):
        return self.get_image()

if __name__ == "__main__":
    from neu_ro_arm.camera.gui import ShowCubes
    cam = Camera()
    cam.gui.show(modifier_fns=[ShowCubes(cam.unpack_configs(False))])
