import numpy as np
import cv2
import threading
import time

from camera.camera_utils import *
import constants as constants

class Capturer:
    #TODO: add error handling if connection is dropped
    def __init__(self):
        self._lock = threading.Lock()
        self._started = False
        self._frame_rate = constants.frame_rate

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
        self._modifer_fns = []

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
        use_live = img is None

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
        return -1

    def add_modifiers(self, modifier_fns=[]):
        if not isinstance(modifier_fns, list):
            modifier_fns = [modifier_fns]
        with self._lock:
            self._modifer_fns.extend(modifier_fns)

    def clear_modifiers(self):
        with self._lock:
            self._modifer_fns = []

    def _update(self):
        while self._showing:
            frame = self._cap.read()
            original_img = frame.copy()
            for fn in self._modifer_fns:
                with self._lock:
                    fn(original_img, frame)

            cv2.imshow(self._window_name, frame)
            k = cv2.waitKey(int(1000/self._cap._frame_rate))
            with self._lock:
                self._last_keypress = k

            #if k == 27: # ESC 
                #self._showing = False
                #cv2.destroyAllWindows()

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

        # self.calc_location()

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

    @property
    def frame_rate(self):
        return self.cap._frame_rate

    def _calc_distortion_matrix(self):
        mtx, newcameramtx, roi, dist = calc_distortion_matrix()
        self._write_configs({
            'mtx' : mtx.tolist(),
            'undistort_mtx' : newcameramtx.tolist(),
            'undistort_roi' : roi,
            'dist_coeffs' : dist.tolist(),
        })

    def _calc_location(self):
        print('Calculating location of camera...')
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

            mtx = self._configs['mtx']
            dist = self._configs['dist_coeffs']

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)

            new_configs = {
                'rvec' : rvecs,
                'tvec' : tvecs,
                'world2cam' : transformation_matrix(rvecs, tvecs),
                'cam2world' : inverse_transformation_matrix(rvecs, tvecs),
            }
            self._configs = self._write_configs(new_configs)
            # return

        # print('ERROR: checkerboard pattern was not identified.')
            # imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            import matplotlib.pyplot as plt

            fig = plt.figure()
            ax = plt.axes(projection='3d')
            origin = np.zeros(3)
            axis = 20*np.array(((1,0,0),(0,1,0),(0,0,1)))
            axis_colors = 'gbr'
            for a, c in zip(axis, axis_colors):
                x,y,z = zip(origin, a)
                ax.plot(x,y,z,'-', color=c)

            # cam_rot_mat, jcb = cv2.Rodrigues(-rvecs)
            cam_axis = coord_transform(self._configs['cam2world'],
                                       axis)
            cam_origin = coord_transform(self._configs['cam2world'],
                                         origin)[0]
            for a, c in zip(cam_axis, axis_colors):
                x,y,z = zip(cam_origin, a)
                ax.plot(x,y,z,'-', color=c)

            ax.plot(*objp.T, 'k.')

            plt.show()

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

    def wait_for_gui(self):
        while self.gui._showing:
            k = self.gui.get_last_keypress()
            if k == 27:
                break

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
        return project_world2pixel(pts_wframe,
                                   self._configs['world2cam'],
                                   self._configs['rvec'],
                                   self._configs['tvec'],
                                   self._configs['mtx'],
                                   self._configs['dist_coeffs'])

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

    def show_feed(self):
        self.gui.show_async()

    def hide_feed(self):
        self.gui.hide()

    def get_image(self, raw=True):
        img = self.cap.read()
        if raw:
            return img
        return self._undistort(img)

    def __call__(self):
        return self.get_image()

    def get_world_pose(self):
        pos = self._configs['world2cam'][3,:3]
        rot_mat = self._configs['world2cam'][:3,:3]
        rot_euler = rotmat2euler(rot_mat)
        return pos, rot_euler

def show_apriltags(original, canvas):
    tags = find_apriltags(original, cam_mtx)
    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(canvas,
                  tuple(tag.corners[idx-1, :].astype(int)),
                  tuple(tag.corners[idx, :].astype(int)),
                  (0, 255, 0))

        text_org = tuple(np.mean(tag.corners, axis=0).astype(int))
        cv2.putText(canvas, str(tag.tag_id),
                    org=text_org,
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.6,
                    thickness=2,
                    color=(0, 0, 255))

def show_cubes(original, canvas):
    cubes = find_cubes(original, cam_mtx, dist_coeffs, cam2world)
    for cube in cubes:
        vert_px = project_world2pixel(cube['vertices'],
                                      world2cam,
                                      rvec,
                                      tvec,
                                      undistort_mtx,
                                      dist_coeffs
                                     ).astype(int)
        for a, b in constants.cube_edges:
            cv2.line(canvas, tuple(vert_px[a]), tuple(vert_px[b]), (255, 0, 0),
                    thickness=2)

def show_arucotags(original, canvas):
    tags = find_arucotags(original, cam_mtx, dist_coeffs)
    for tag in tags:
        corners = tag['corners']
        for c_id in range(len(corners)):
            cv2.line(canvas,
                  tuple(corners[c_id-1, :].astype(int)),
                  tuple(corners[c_id, :].astype(int)),
                  (0, 255, 0))

        text_org = tuple(np.mean(corners, axis=0).astype(int))
        cv2.putText(canvas, str(tag['tag_id']),
                    org=text_org,
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.5,
                    thickness=1,
                    color=(0, 0, 255))

def show_cam_z_vec(original, canvas):
    z_vec = np.array(((0,0,1.),(0,0.4,1.)))
    z_vec = coord_transform(cam2world, z_vec)
    px = project_world2pixel(z_vec, world2cam, rvec, tvec,
                             undistort_mtx, dist_coeffs)
    for p in px:
        cv2.circle(canvas, tuple(p.astype(int)), 2, (255,0,0), thickness=2)


if __name__ == "__main__":
    camera = Camera(2)
    #img = camera.get_image()

    camera.show_feed()
    camera.wait_for_gui()
    camera._calc_location()
    exit()
    cam_mtx = camera._configs['undistort_mtx'].copy()
    world2cam = camera._configs['world2cam']
    cam2world = camera._configs['cam2world']
    rvec = camera._configs['rvec']
    tvec = camera._configs['tvec']
    undistort_mtx = camera._configs['undistort_mtx']
    dist_coeffs = camera._configs['dist_coeffs']

    # camera.gui.add_modifiers(show_arucotags)
    camera.gui.add_modifiers(show_cubes)
    # camera.gui.add_modifiers(show_cam_z_vec)
    camera.show_feed()
    camera.wait_for_gui()
    exit()

    camera.show_feed()
    # camera.gui.add_modifiers(show_cubes)
    camera.gui.add_modifiers(show_apriltags)
    time.sleep(100)
    camera.hide_feed()

