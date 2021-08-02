import cv2
import numpy as np
import threading
import time
import pybullet as pb

import nuro_arm.constants as constants

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

        time.sleep(5)# time.sleep(delay) delay wasn't defined
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
    def __init__(self, pb_client):
        self._pb_client = pb_client
        self.view_mtx = None
        self.projection_mtx = None

    def read(self):
        img_width = 640
        img_height = 480
        return pb.getCameraImage(width=img_width,
                                  height=img_height,
                                  viewMatrix = self.view_mtx,
                                  projectionMatrix=self.projection_mtx,
                                  physicsClientId=self._pb_client
                                 )[2]
