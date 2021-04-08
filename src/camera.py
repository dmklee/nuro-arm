import numpy as np
import cv2
import threading
import time

from src.camera_utils import *
import src.constants as constants

class Capturer:
    #TODO: add error handling if connection is dropped
    def __init__(self):
        self._lock = threading.Lock()
        self._started = False
        self._frame_rate = 20

    def set_camera_id(self, camera_id, run_async=True):
        self._camera_id = camera_id
        self._cap = cv2.VideoCapture(camera_id)
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
            self.thread.join()

    def __call__(self):
        return self.read()

    def __del__(self):
        self.stop_async()
        if self._cap.isOpened():
            self._cap.release()

class GUI:
    def __init__(self, capturer):
        self._lock = threading.Lock()
        self._showing = False
        self._cap = capturer
        self._window_name = 'gui'

    def show_async(self, window_name=None):
        if self._showing:
            self.hide()

        if window_name is not None:
            self.change_window_name(window_name)
        self._showing = True
        self._last_keypress = -1
        self.thread = threading.Thread(target=self._update,
                                       args=(),
                                       daemon=True)
        self.thread.start()

    def show(self, img=None, window_name='', exit_keys=[]):
        if img is None:
            use_live = True
        k = -1
        while True:
            if use_live:
                img = self._cap.read()
            cv2.imshow(window_name, img)
            k = cv2.waitKey(int(1000/self._cap._frame_rate))
            if k == 27 or k in exit_keys:
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
        cv2.destroyAllWindows()
        return k

    def change_window_name(self, name):
        with self._lock:
            cv2.destroyWindow(self.change_window_name)
            self._window_name = name

    def get_last_keypress(self):
        if self._showing:
            with self._lock:
                k = self._last_keypress
        return k

    def _update(self):
        while self._showing:
            frame = self._cap.read()
            cv2.imshow(self._window_name, frame)
            k = cv2.waitKey(int(1000/self._cap._frame_rate))
            with self._lock:
                self._last_keypress = k

            if k == 27: # ESC 
                self.hide()

    def hide(self):
        if self._showing:
            self._showing = False
            self.thread.join()
            cv2.destroyAllWindows()

class Camera:
    CONFIG_FILE = "src/configs/camera.npz"
    def __init__(self,
                 camera_id=None,
                 ):
        self.cap = Capturer()
        self.gui = GUI(self.cap)

        self._configs = self._read_configs()

        is_connected = self.connect(camera_id)
        if not is_connected:
            return

        # self._calc_distortion_matrix()

        self._get_ego_pose()

    def connect(self, camera_id=None):
        if camera_id is None:
            print('Camera id not specified, searching for available cameras...')
            for c_id in range(5):
                is_valid = self.cap.set_camera_id(c_id)
                if is_valid:
                    name = f"Camera{c_id}: is this the camera you want to use? [y/n]"
                    k = self.gui.show(window_name=name,
                                      exit_keys=[ord('y'), ord('n')]
                                     )
                    if k == ord('y'):
                        print(f'Video capture enabled with camera{c_id}.')
                        return True
                else:
                    print(f'  Camera{c_id} not available.')
            print('[ERROR] No other cameras were found.')
            return False

        is_valid = self.cap.set_camera_id(camera_id)
        if is_valid:
            print(f'Video capture enabled with camera{camera_id}.')
            return True

        print(f'Video capture failed with camera{camera_id}.')
        return False

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

    def _undistort(self, img):
        # undistort
        if 'mtx' not in self._configs:
            return img

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

        return configs

    def _read_configs(self):
        configs = dict(np.load(self.CONFIG_FILE))

        if configs is None:
            configs = dict()

        return configs

    def get_image(self):
        img = self.cap.read()
        return self._undistort(img)

    def __call__(self):
        return self.get_image()

if __name__ == "__main__":
    camera = Camera(0)
    camera.start_recording(5, 1)
    r = camera.wait_for_recording()
    for i in range(len(r)):
        cv2.imshow('sdaf', r[i])
        cv2.waitKey(30)
    cv2.destroyAllWindows()

