import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import yaml
import time

#TODO:
#   - handle error on connection, default to attempt reconnecting
#   - write calibration for lens distortion, this can probably be held for all cameras, and will not be overwritten
#   - write calc_camera_pose

class Camera:
    def __init__(self,
                 camera_id=0,
                 ):
        self._cam_id = camera_id

        self._cam = self._connect()

        self._configs = self._read_configs()
        for k,v in self._configs.items():
            if isinstance(v, list):
                self._configs[k] = np.array(v)

        # self._get_camera_pose()

    def set_camera_id(self, camera_id):
        self._cam_id = camera_id
        self._cam = self._connect()

    def _connect(self):
        return cv2.VideoCapture(self._cam_id)

    def get_image(self):
        ret, image = self._cam.read()
        if not ret:
            print('error, try reconnecting')
            return

        return image

    def reshape_image(self, image, height, width=None):
        if width is None:
            width = height
        reshaped = cv2.resize(image, (width, height))
        return reshaped

    def rescale_image(self, image, scale):
        new_height = int(image.shape[0] * scale)
        new_width = int(image.shape[1] * scale)
        return self.reshape_image(image, new_height, new_width)

    def convert_gray(self, image):
        return image.mean(axis=2, keepdims=True)

    def _disconnect(self):
        self._cam.release()

    def _calc_distortion_matrix(self):
        folder = "camera/checkerboard_calibration/"
        positions = np.loadtxt(os.path.join(folder, "3dpositions.txt"),
                                delimiter=',')
        num_images = len(positions)

        GW, GH = 7, 9
        grid_size = 20 #mm
        objp = np.zeros((GW*GH,3), np.float32)
        objp[:,:2] = grid_size * np.mgrid[0:GW,0:GH].T.reshape(-1,2)

        img_pts = []
        obj_pts = []
        for img_id in range(num_images):
            img = cv2.imread(os.path.join(folder,f"{img_id}.jpg"))
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (GW,GH), None)

            if ret:
                # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                # corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
                img_pts.append(corners)
                obj_pts.append(objp)

        cv2.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, gray.shape[::-1], None, None)

        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        self._write_configs({
            'mtx' : mtx.tolist(),
            'undistort_mtx' : newcameramtx.tolist(),
            'undistort_roi' : roi,
            'dist_coeffs' : dist.tolist(),
        })

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

    def _read_configs(self):
        with open("camera/configs.yaml") as f:
            doc = yaml.safe_load(f)

        if doc is None:
            doc = dict()

        return doc

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
    camera = Camera()
    camera._calc_distortion_matrix()
    camera.live_feed()

