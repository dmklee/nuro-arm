import numpy as np
import cv2
import threading
import time

from neu_ro_arm.camera.camera_utils import *
import neu_ro_arm.constants as constants
from neu_ro_arm.camera.gui import GUI

class Capturer:
    #TODO: add error handling if connection is dropped
    def __init__(self):
        self._lock = threading.Lock()
        self._started = False
        self._frame_rate = constants.frame_rate
        self._cap = None

    def set_camera_id(self, camera_id, run_async=True):
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

    def start_recording(self, duration, delay=0):
        self.end_recording()

        n_frames = self._frame_rate * duration
        self._record_buffer = np.empty((n_frames, *self._img_shape),
                                       dtype=np.uint8)
        self._record_id = 0

        time.sleep(delay)
        with self._lock:
            self._recording = True

    def end_recording(self):
        with self._lock:
            self._recording = False

    def is_recording(self):
        with self._lock:
            is_recording = self._recording

        return is_recording

    def get_recording(self):
        if self._recording:
            print('recording is still in progress')
            return None
        with self._lock:
            recording = self._record_buffer.copy()
        return recording

    def read(self):
        if self._started:
            with self._lock:
                ret = self._ret
                frame = self._frame.copy()
        else:
            ret, frame = self._cap.read()

        return frame if ret else None

    def set_frame_rate(self, frame_rate):
        with self._lock:
            self._frame_rate = frame_rate

    def stop_async(self):
        if self._started:
            self._started = False
            self._recording = False
            self.thread.join()

    def __call__(self):
        return self.read()

    def release(self):
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

    def _snapshot(self):
        return getCameraImage(width=img_width,
                              height=img_height,
                              viewMatrix = self.view_mtx,
                              projectionMatrix=self.projection_mtx,
                              physicsClientId=self._pb_client
                             )[2]

class Camera:
    CONFIG_FILE = "neu_ro_arm/camera/configs.npz"
    def __init__(self,
                 camera_id=None,
                 ):
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
        is_connected = self.cap.set_camera_id(self._camera_id)
        return is_connected

    @property
    def frame_rate(self):
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

    def start_recording(self, duration, delay):
        self.cap.start_recording(duration, delay)

    def end_recording(self):
        self.cap.end_recording()

    def get_recording(self):
        return self.cap.get_recording()

    def wait_for_recording(self):
        while self.cap.is_recording():
            time.sleep(0.1)
        return self.get_recording()

    def undistort(self, img):
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
        return project_world2pixel(pts_wframe,
                                   self._configs['world2cam'],
                                   self._configs['rvec'],
                                   self._configs['tvec'],
                                   self._configs['mtx'],
                                   self._configs['dist_coeffs'])

    def _update_config_file(self, new_configs):
        # get existing configs
        configs = np.load(self.CONFIG_FILE)
        configs = dict(configs) if configs is not None else dict()

        configs.update(new_configs)
        np.savez(self.CONFIG_FILE, **configs)

    def unpack_configs(self):
        configs = dict(np.load(self.CONFIG_FILE))

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
            print('[ERROR] Some camera configs are missing. Run setup_camera.py '
                  'to properly populate config file.')

    def get_image(self):
        img = self.cap.read()
        return img

    def __call__(self):
        return self.get_image()

    def get_world_pose(self):
        pos = self._configs['world2cam'][3,:3]
        rot_mat = self._configs['world2cam'][:3,:3]
        rot_euler = rotmat2euler(rot_mat)
        return pos, rot_euler
