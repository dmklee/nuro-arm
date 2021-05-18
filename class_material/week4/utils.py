import cv2
from threading import Thread
import numpy as np

from neu_ro_arm.camera import camera_utils
from neu_ro_arm.camera.gui import ImageModifierFunction

face_detector = cv2.CascadeClassifier('class_material/week4/haarcascade_frontalface_default.xml')

def get_face(img):
    '''Detects location of face in image

    Parameters
    ----------
    img : array_like
        rgb or grayscale image

    Returns
    -------
    x : int
        horizontal position of face in pixel space
    y : int
        vertical position of face in pixel space
    w : int
        horizontal size of face in pixel space
    h : int
        vertical size of face in pixel space
    '''
    gray = camera_utils.convert_gray(img)
    faces, _, confidences = face_detector.detectMultiScale3(gray,
                                           scaleFactor=1.1,
                                           minNeighbors=7,
                                           minSize=(50,50),
                                           outputRejectLevels=True
                                          )
    if len(faces) == 0:
        return None

    #get max level_weights
    max_id = np.argmax(np.array(confidences).flatten())
    x,y,w,h = faces[max_id]
    return int(x+w/2), int(y+h/2), w, h

class ShowFace(ImageModifierFunction):
    def __call__(self, original, canvas):
        face_data = get_face(original)
        if face_data is not None:
            x,y,w,h = face_data
            canvas = cv2.ellipse(canvas, (x, y), (int(w/2), int(h/2)),
                                 0, 0, 360, color=(0,0,255), thickness=4)
        return canvas

class AsyncMover(Thread):
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        pass
