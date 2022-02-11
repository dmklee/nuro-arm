import numpy as np

from nuro_arm import Camera
from nuro_arm.constants import CAMERA_CONFIG_FILE
from nuro_arm.camera.gui import ShowCheckerboard
from nuro_arm.tk_utils import Popup, VideoPopup, ImagePopup, Colors

def calibrate_camera():
    '''Scans over available cameras, prompting user to select which camera is
    to be used with the robot
    '''
    popup = Popup(
        title='Camera Set-up: step 0 of 2',
        text='Setting up the camera will take less than a minute. Please ensure \n' \
             'that the camera is plugged into your computer and the lens cover is \n' \
             'removed. \n\n' \
             'Press BEGIN to start the set-up process.',
        button_names=['BEGIN', 'QUIT']
    )
    if popup.response() != 'BEGIN':
        exit()

    # Determine the ID of the camera
    cam_id = 0
    camera = Camera(camera_id=cam_id)
    while True:
        if cam_id > 10:
            popup = Popup(
                title='Camera Warning',
                text='No more cameras have been found.  Double-check that the \n' \
                     'camera is plugged in to your computer and try again.',
                text_color=Colors.ALARM,
                button_names=['CLOSE'],
                button_colors=[Colors.NEUTRAL]
            )()
            exit()

        if camera.change_camera_id(cam_id):
            img = camera.get_image()
            popup = ImagePopup(
                title='Camera Set-up: step 1 of 2',
                text='First, we will make sure that we have selected the correct camera. \n\n' \
                     'If this appears to be the right camera feed, click YES.  Otherwise \n' \
                     'click NO to search for another available camera. \n\n',
                button_names=['YES', 'NO', 'QUIT'],
                images=[img],
                image_shape=(300,400),
                button_colors=[Colors.YES, Colors.NEUTRAL, Colors.NO]
            )
            if popup.response() == 'YES':
                break
            elif popup.response() != 'NO':
                exit()

        cam_id += 1

    class HighlightedCheckerboard:
        def __init__(self, camera):
            self.camera = camera
            self.modifier = ShowCheckerboard({})

        def read(self):
            img = self.camera.get_image()
            return self.modifier(img, img)

    popup = VideoPopup(
        HighlightedCheckerboard(camera),
        title='Camera Set-up: step 2 of 2',
        text='Now, we will determine the location of the camera in relation to the \n' \
             'robot.  Place the checkerboard calibration pattern in front of the robot \n' \
             '(e.g. opposite the control board) such that the suction cups are aligned\n' \
             'with the corresponding semicircles on the page. Angle the camera such that  \n' \
             'the pattern fits fully within the image then tighten the screws on the \n' \
             'camera stand to ensure so that it does not rotate. \n\n' \
             'Click CONTINUE once you are ready (there should be rainbow highlighting).\n\n',
        button_names=['CONTINUE', 'QUIT'],
        image_shape=(375,500),
        button_colors=[Colors.YES, Colors.NO]
    )
    if popup.response() != 'CONTINUE':
        exit()

    ret, location_data = camera.calc_location()
    if not ret:
        popup = Popup(
            title='Camera Warning',
            text='Camera failed to identify checkerboard pattern. Ensure it is \n' \
                 'completely located within image and try again.',
            text_color=Colors.ALARM,
            button_names=['CLOSE'],
            button_colors=[Colors.NEUTRAL]
        )()
        exit()

    rvec, tvec, world2cam, cam2world = location_data
    data = {'camera_id': cam_id,
            'rvec' : rvec,
            'tvec' : tvec,
            'world2cam' : world2cam,
            'cam2world' : cam2world,
           }
    np.save(CAMERA_CONFIG_FILE, data)

    popup = Popup(
        title='Success',
        text='Camera was successfully calibrated. All data has been logged\n' \
             'and you can now use the camera for finding object poses.',
        button_names=['CLOSE'],
        button_colors=[Colors.NEUTRAL]
    )()

if __name__ == "__main__":
    calibrate_camera()
