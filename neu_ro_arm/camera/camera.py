import numpy as np
import cv2
import threading
import time

from neu_ro_arm.camera.camera_utils import *
import neu_ro_arm.constants as constants
from neu_ro_arm.camera.gui import GUI

class Capturer:
    def __init__(self):
        '''Class to allow asynchronous video capture from camera
        '''
        self._lock = threading.Lock()
        self._started = False
        self._frame_rate = constants.frame_rate
        self._cap = None

    def set_camera_id(self, camera_id, run_async=True):
        '''Changes video capture to a different camera id number

        Captured images are forced to a height of 480, width of 640. This class
        can only handle a single camera at a time, so it will release an existing
        connection before creating the new one.

        Parameters
        ----------
        camera_id: int
            Camera number used to open cv2.VideoCapture instance
        run_async: bool, default True
            flag indicating whether another thread should be spawned to capture
            frames asynchronously from the camera

        Returns
        -------
        bool
            flag indicating if a connection was made to the camera.  a value of
            False suggests either an invalid camera id or the camera is already 
            being used by another application
        '''
        # release any old connections
        self.release()

        self._camera_id = camera_id
        self._cap = cv2.VideoCapture(camera_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        connected = self._cap.isOpened()
        if connected and run_async:
            self.start_async()
        return connected

    def start_async(self):
        '''Launches thread that captures frames from the camera.

        Resets all settings so recordings will be lost
        '''
        if self._started:
            # we need to stop before starting a new one
            self.stop_async()

        self._started = True
        self._ret, self._frame = self._cap.read()
        self._img_shape = self._frame.shape
        self._recording = False
        self._record_buffer = None
        self._record_id = None

        self.thread = threading.Thread(target=self._update,
                                       args=(),
                                       daemon=True)
        self.thread.start()

    def _update(self):
        '''Asynchronous reading of frames from camera, handles recording to buffer
        '''
        while self._started:
            ret, frame = self._cap.read()
            with self._lock:
                self._ret = ret
                self._frame = frame
                if self._recording:
                    if self._record_id < len(self._record_buffer):
                        self._record_buffer[self._record_id] = frame.copy()
                        self._record_id += 1
                    else:
                        self._recording = False
            time.sleep(1./self._frame_rate)

    def start_recording(self, duration):
        '''Begins recording frames from camera

        Any existing recording will be ended and immediately overwritten.  The
        number of frames taken is based on the frame rate

        Parameters
        ----------
        duration: float
            Seconds of recording to take
        '''
        self.end_recording()

        n_frames = self._frame_rate * duration
        self._record_buffer = np.empty((n_frames, *self._img_shape),
                                       dtype=np.uint8)
        self._record_id = 0

        time.sleep(delay)
        with self._lock:
            self._recording = True

    def end_recording(self):
        '''Stops recording frames in during thread update
        '''
        with self._lock:
            self._recording = False

    def is_recording(self):
        '''Checks if thread is recording frames

        Returns
        -------
        bool
            True if currently recording frames in thread, False otherwise
        '''
        with self._lock:
            is_recording = self._recording

        return is_recording

    def get_recording(self):
        '''Returns recorded frames if recording is over. Returns None if recording
        still in progress.

        Returns
        -------
        ndarray
            sequence of images
        '''
        with self._lock:
            recording = self._record_buffer.copy()
        return recording

    def read(self):
        '''Reads last frame from camera

        Returns None if camera failed to return a frame.

        Returns
        -------
        ndarray
            sequence of images
        '''
        if self._started:
            with self._lock:
                ret = self._ret
                frame = self._frame.copy()
        else:
            ret, frame = self._cap.read()

        return frame if ret else None

    def set_frame_rate(self, frame_rate):
        '''Changes frame rate used by asynchronous thread

        Parameters
        ----------
        frame_rate: int
            Number of frames to capture per second
        '''
        with self._lock:
            self._frame_rate = frame_rate

    def stop_async(self):
        '''Closes thread that captures frames asynchronously
        '''
        if self._started:
            self._started = False
            self._recording = False
            self.thread.join()

    def __call__(self):
        return self.read()

    def release(self):
        '''Closes connection to current camera if it is open
        '''
        self.stop_async()
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()

    def __del__(self):
        self.release()

class SimCapturer(Capturer):
    #TODO: find a good abstraction to unite the interface of simulated and real camera
    # i think we just need to create a simulator camera "cap" class that has method
    # "read"
    img_width = 640
    img_height = 480
    def __init__(self, pb_client, camera_pose):
        self._pb_client = pb_client
        self.view_mtx
        self.projection_mtx

    def read(self):
        return getCameraImage(width=img_width,
                              height=img_height,
                              viewMatrix = self.view_mtx,
                              projectionMatrix=self.projection_mtx,
                              physicsClientId=self._pb_client
                             )[2]

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
        mtx, newcameramtx, roi, dist = calc_distortion_matrix()
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
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

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

            world2cam = transformation_matrix(rvec, tvec)
            cam2world = inverse_transformation_matrix(rvec, tvec)

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
            sequence of 2D pixel indices
        '''
        # transform world to camera frame
        return project_world2pixel(pts_wframe,
                                   self._world2cam,
                                   self._rvec,
                                   self._tvec,
                                   self._mtx,
                                   self._dist_coeffs)

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
                self._camera_id = configs['camera_id']
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

    def __call__(self):
        return self.get_image()
