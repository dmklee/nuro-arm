import cv2
import numpy as np
import threading
import time
import pybullet as pb

from nuro_arm.constants import FRAME_RATE, DEFAULT_CAM_POSE_MTX, CAM_MTX, CAM_DIST_COEFFS
from nuro_arm.transformation_utils import invert_transformation, coord_transform


class Capturer:
    def __init__(self, img_width=640, img_height=480):
        '''Class to allow asynchronous video capture from camera

        Parameters
        ----------
        img_width : int, default to 640
            Width of image to be captured in number of pixels
        img_height : int, default to 480
            Height of image to be captured in number of pixels
        '''
        self._lock = threading.Lock()
        self._started = False
        self._frame_rate = FRAME_RATE
        self._cap = None
        self._img_width = img_width
        self._img_height = img_height

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
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._img_width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._img_height)
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
        self._ret, self._frame = self._get_feed()
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
            ret, frame = self._get_feed()
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
            ret, frame = self._get_feed()

        return frame if ret else None

    def _get_feed(self):
        return self._cap.read()

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

    @property
    def camera_mtx(self):
        return CAM_MTX

    @property
    def dist_coeffs(self):
        return CAM_DIST_COEFFS

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
    def __init__(self, img_width=320, img_height=240, fov=50,
                 view_mtx=None, pb_client=0):
        '''Class to allow asynchronous video capture within pybullet simulator

        Parameters
        ----------
        img_width : int, default to 640
            Width of image to be captured in number of pixels
        img_height : int, default to 480
            Height of image to be captured in number of pixels
        fov : float, default to 50
            Field of view in degrees in horizontal direction, fov in vertical
            direction is calculated based on img_width & img_height
        view_mtx : array_like, optional
            4x4 view matrix that describes location of camera in world frame.
            If not provided, the default camera pose (from nuro_arm.constants)
            will be used
        pb_client : int, default to 0
            Physics Client ID describing which simulator the camera exists
            within.  If you are only using one simulator, you can leave as
            default.
        '''
        super().__init__(img_width, img_height)
        self.set_fov(fov)
        self._pb_client = pb_client

    def set_pose_mtx(self, pose_mtx=None):
        '''Re-position camera in simulator

        Parameters
        ----------
        pose_mtx : array_like, optional
            4x4 pose matrix that describes location of
            camera in world frame.  If not provided, the default camera pose
            (from nuro_arm.constants) will be used
        '''
        if pose_mtx is None:
            print('[WARNING:] no pose matrix specified for simulated camera. '
                  'Camera will be placed in default location.')
            pose_mtx = DEFAULT_CAM_POSE_MTX

        vecs = np.array(((0,0,0), (0,0,1), (0,-1,0)))
        eye_pos, target_pos, up_vec = coord_transform(pose_mtx, vecs)
        view_mtx = pb.computeViewMatrix(eye_pos, target_pos, up_vec)

        self._view_mtx = view_mtx

    def set_fov(self, fov):
        '''Change camera's field of view

        Parameters
        ----------
        fov : float, default to 50
            Field of view in degrees
        '''
        self._fov = fov
        aspect = self._img_width/self._img_height
        self._proj_mtx = pb.computeProjectionMatrixFOV(fov, aspect, 0.05, 2)

    def _get_feed(self):
        ret = True
        img = pb.getCameraImage(width=self._img_width,
                                height=self._img_height,
                                viewMatrix=self._view_mtx,
                                projectionMatrix=self._proj_mtx,
                                renderer=pb.ER_TINY_RENDERER,
                                physicsClientId=self._pb_client
                                )[2][...,:3]

        return ret, img

    def set_camera_id(self, camera_id=0, run_async=True):
        self.release()

        if run_async:
            self.start_async()
        return True

    @property
    def camera_mtx(self):
        fov_radians = np.radians(self._fov)
        aspect = self._img_width/self._img_height

        mtx = np.zeros((3,3))
        mtx[0,0] = 0.5 * self._img_height / np.tan(fov_radians/2)
        mtx[1,1] = 0.5 * self._img_width / np.tan(aspect * fov_radians/2)
        mtx[0,2] = self._img_width / 2
        mtx[1,2] = self._img_height / 2
        mtx[2,2] = 1
        return mtx

    @property
    def dist_coeffs(self):
        return np.zeros((1,5))

    def release(self):
        '''Closes connection to current camera if it is open
        '''
        self.stop_async()
