import numpy as np
import cv2
from abc import abstractmethod

class GUI:
    def __init__(self, capturer):
        self._cap = capturer

    def show(self, img=None, window_name='',
             exit_keys=[],
             modifier_fns=[]):
        use_live = img is None

        k = -1
        cv2.namedWindow(window_name)
        cv2.startWindowThread()
        cv2.setWindowProperty(window_name,
                              cv2.WND_PROP_TOPMOST, 1)
        while True:
            if use_live:
                img = self._cap.read()

            canvas = img.copy()
            for mod_fn in modifier_fns:
                canvas = mod_fn(canvas, img)

            cv2.imshow(window_name, img)
            k = cv2.waitKey(int(1000/self._cap._frame_rate))
            if k == 27 or k in exit_keys:
                break

        # wait key is needed to get window to close on Mac
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)

        return k

class GUIModifierFunction:
    def __init__(self, **kwargs):
        self.cam_configs = kwargs

    @abstractmethod
    def __call__(self, canvas, original):
        return canvas

class ShowCubes(GUIModifierFunction):
    def __call__(self, canvas, original):
        cubes = find_cubes(original,
                           self.cam_configs['cam_mtx'],
                           self.cam_configs['dist_coeffs'],
                           self.cam_configs['cam2world'])
        for cube in cubes:
            vert_px = project_world2pixel(cube['vertices'],
                                          self.cam_configs['world2cam'],
                                          self.cam_configs['rvec'],
                                          self.cam_configs['tvec'],
                                          self.cam_configs['undistort_mtx'],
                                          self.cam_configs['dist_coeffs'],
                                         ).astype(int)
            for a, b in constants.cube_edges:
                canvas = cv2.line(canvas, tuple(vert_px[a]), tuple(vert_px[b]),
                                  (255, 0, 0), thickness=2)

        return canvas

class ShowArucoTags(GUIModifierFunction):
    def __call__(original, canvas):
        tags = find_arucotags(original,
                              self.cam_configs['cam_mtx']
                              self.cam_configs['dist_coeffs']
                             )
        for tag in tags:
            corners = tag['corners']
            for c_id in range(len(corners)):
                canvas = cv2.line(canvas,
                                  tuple(corners[c_id-1, :].astype(int)),
                                  tuple(corners[c_id, :].astype(int)),
                                  (0, 255, 0)
                                 )

            text_org = tuple(np.mean(corners, axis=0).astype(int))
            canvas = cv2.putText(canvas, str(tag['tag_id']),
                                org=text_org,
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.5,
                                thickness=1,
                                color=(0, 0, 255)
                                )
    return canvas
