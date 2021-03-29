import numpy as np
import matplotlib.pyplot as plt
import cv2

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

        self._calc_distortion_data()

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
        image = self.get_image()
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (7,9), None)

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            print(corners.shape)

        plt.figure()
        plt.imshow(gray, cmap='gray')
        for x,y in corners[:,0]:
            plt.plot(x-0.5,y-0.5,'r.')
        plt.axis('off')
        plt.show()

    def _load_distortion_data(self):
        print('loading non existant distortion data')
        pass

    def __del__(self):
        self._disconnect()

def use_as_webcam(cam_id=0):
    cam = cv2.VideoCapture(cam_id)

    cv2.namedWindow("test")

    img_counter = 0

    while True:
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("test", frame)

        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k%256 == 32:
            # SPACE pressed
            img_name = "opencv_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1

    cam.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = Camera()
    # use_as_webcam()

