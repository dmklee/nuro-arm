import numpy as np
import cv2
from abc import abstractmethod

from nuro_arm.camera import camera_utils
import nuro_arm.constants as constants

class GUI:
    def __init__(self, capturer):
        '''Class that displays images or camera feed using cv2 gui

        Parameters
        ----------
        capturer : obj
            instance of Capturer class
        '''
        self._cap = capturer

    def show(self,
             img=None,
             window_name='press ESC to close',
             exit_keys=[],
             modifier_fns=[]
            ):
        '''Show an image or feed until exit keys are pressed

        Parameters
        ----------
        img : array_like, optional
            single image to be displayed; if not provided, the feed from the
            capturer will be used
        window_name : str
            title for the window that displays image
        exit_keys : list of int
            list of keys that will close window when pressed; keys are represented
            as the integer corresponding to the unicode character; for instance,
            if we want to close on 'y' and 'n', then we would use
            exit_keys=[ord('y'),ord('n')]
        modifier_fns : list of obj
            list of GUIModifierFunction instances that will be called in order
            to modify the image being displayed

        Returns
        -------
        int
            integer representing last unicode character detected, a value of -1
            means no key press was detected
        '''
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

            cv2.imshow(window_name, canvas)
            k = cv2.waitKey(int(1000/self._cap._frame_rate))
            if k == 27 or k in exit_keys:
                break

        # wait key is needed to get window to close on Mac
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)

        return k

class ImageModifierFunction:
    def __init__(self, cam2world):
        '''Function that performs some image operations and adds modifications
        for debugging/visualization purposed

        Must implement __call__ method which will be used by GUI for plotting

        Parameters
        ----------
        cam2world : np.ndarray
        '''
        self.cam2world = cam2world

    @abstractmethod
    def __call__(self, canvas, original):
        '''Modifies canvas image

        Parameters
        ----------
        canvas : ndarray
            image to be modified; it may already have some modifications to it
        original : ndarray
            image that has not been modified; any processing should be done on
            this image so the results are not messed up by previous modifications

        Returns
        -------
        ndarray
            canvas image plus some additional modifications
        '''
        return canvas


class ShowCubes(ImageModifierFunction):
    def __init__(self, cam2world, cube_size=None,
                 tag_size=None, include_id=False):
        super().__init__(cam2world)
        self.include_id = include_id
        self.cube_size = cube_size
        self.tag_size = tag_size

    def __call__(self, original, canvas=None):
        '''Draws wireframe models for all cubes detected in the image via
        aruco tag detection
        '''
        if canvas is None:
            canvas = np.array(original)

        cubes = camera_utils.find_cubes(original, self.cam2world)

        for cube in cubes:
            vert_px = camera_utils.project_to_pixels(cube.vertices,
                                                     self.cam2world
                                                    ).astype(int)

            for a, b in constants.CUBE_EDGES:
                canvas = cv2.line(canvas, tuple(vert_px[a]), tuple(vert_px[b]),
                                  (255, 0, 0), thickness=2)

            if self.include_id:
                text_org = tuple(np.mean(vert_px, axis=0).astype(int))
                canvas = cv2.putText(canvas, str(cube.id_),
                                     org=text_org,
                                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                     fontScale=0.8,
                                     thickness=2,
                                     color=(0, 0, 255))

        return canvas

class ShowArucoTags(ImageModifierFunction):
    def __init__(self, cam2world, tag_size=None):
        super().__init__(cam2world)
        self.tag_size = tag_size

    def __call__(self, original, canvas=None):
        '''Draws tag outlines and ids for all aruco tags detected in the image
        '''
        if canvas is None:
            canvas = np.array(original)
        tags = camera_utils.find_arucotags(original, tag_size=self.tag_size)
        for tag in tags:
            for c_id in range(4):
                canvas = cv2.line(canvas,
                                  tuple(tag.corners[c_id-1].astype(int)),
                                  tuple(tag.corners[c_id].astype(int)),
                                  (0, 0, 255),
                                  thickness=2,
                                 )

            text_org = tuple(np.mean(tag.corners, axis=0).astype(int))
            canvas = cv2.putText(canvas, str(tag.id_),
                                org=text_org,
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.5,
                                thickness=2,
                                color=(255, 0, 255)
                                )
        return canvas

class ShowFace(ImageModifierFunction):
    def __call__(self, original, canvas=None):
        if canvas is None:
            canvas = np.array(original)
        # highlight center of image
        cx = int(original.shape[1]/2)
        cy = int(original.shape[0]/2)
        canvas = cv2.drawMarker(canvas, (cx,cy), markerType=cv2.MARKER_CROSS,
                                color=(255,0,0), markerSize=14, thickness=2)

        face_data = camera_utils.find_face(original)
        if face_data is not None:
            x,y,w,h = face_data
            canvas = cv2.ellipse(canvas, (x, y), (int(w/2), int(h/2)),
                                 0, 0, 360, color=(0,0,255), thickness=4)
            canvas = cv2.drawMarker(canvas, (x,y), markerType=cv2.MARKER_CROSS,
                                    color=(0,0,255), markerSize=10, thickness=2)

        return canvas

class ShowCheckerboard(ImageModifierFunction):
    def __call__(self, original, canvas):
        # highlight center of image
        ret, corners = cv2.findChessboardCorners(original,
                                                 constants.CALIBRATION_GRIDSHAPE,
                                                 None)
        if ret:
            cv2.drawChessboardCorners(canvas,
                                      constants.CALIBRATION_GRIDSHAPE,
                                      corners,
                                      ret)

        return canvas
