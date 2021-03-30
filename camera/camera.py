import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml
import time

from camera.utils import calc_distortion_matrix
#TODO:
#   - handle error on connection, default to attempt reconnecting
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
        mtx, newcameramtx, roi, dist = calc_distortion_matrix()
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
            img = self._undistort(img)
            cv2.imshow("Live Feed", img)
            cv2.waitKey(int(1000/frate))

        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = Camera()
    camera._calc_distortion_matrix()
    camera.live_feed()

