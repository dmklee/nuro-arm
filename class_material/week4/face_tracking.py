##############################################################################
# Author: David Klee
# Date: May 18, 2021
#
#
##############################################################################
import numpy as np
import cv2
import time
from threading import Thread

from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.camera.camera_utils import find_face
from neu_ro_arm.camera.gui import ShowFace

class FaceFollower(Thread):
    def __init__(self, robot, cam):
        super().__init__()
        self.robot = robot
        self.camera = camera
        self.delay = 0.3 # seconds
        self.running = True
        self.BASE_ID = 0
        self.WRIST_ID = 3

    def run(self):
        while self.running:
            image = self.camera.get_image()

            # detect face in image
            face = find_face(image)
            if face is None:
                # no face found so go to next iteration
                time.sleep(self.delay)
                continue

            # call the function that you wrote
            rel_base_move, rel_wrist_move = follow_face(face, image)

            # issue relative motion commands
            arm_jpos = self.robot.get_arm_jpos()
            arm_jpos[self.BASE_ID] += rel_base_move
            arm_jpos[self.WRIST_ID] += rel_wrist_move
            self.robot.move_arm_jpos(arm_jpos)

def follow_face(detected_face, image):
    '''Returns relative movement commands

    You may want to add print statements to help you out here
    '''
    x, y, w, h = detected_face
    image_height, image_width, _ = image.shape

    rel_wrist_move = 0
    rel_base_move = 0
    #####################################
    # your code goes here
    #####################################

    #####################################
    #####################################
    return rel_base_move, rel_wrist_move

if __name__ == "__main__":
    # initialize the camera
    camera = Camera()

    # initialize the robot
    starting_arm_jpos = [0.0, -0.85, 1.0, 1.24, 0.0]
    robot = RobotArm()
    robot.move_arm_jpos(starting_arm_jpos)

    # begin thread that issues commands based on your code
    follower = FaceFollower(robot, camera)
    follower.start()

    # set up GUI so you can visualize the face in the image
    draw_face = ShowFace(camera.unpack_configs(False))
    camera.gui.show(window_name='Detecting Faces (press ESC to exit)',
                   modifier_fns=[draw_face])

    # close follower so robot stops moving
    follower.running = False
    time.sleep(0.5)
