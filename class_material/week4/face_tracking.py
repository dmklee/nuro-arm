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

def move_base_relative(robot, rel_angle):
    '''Moves base motor by some relative angle (in radians)
    '''
    base_servo_idx = 0
    arm_jpos = robot.get_arm_jpos()

    # adjust the angle of the base servo
    arm_jpos[base_servo_idx] += rel_angle

    # move to adjusted postion
    robot.move_arm_jpos(arm_jpos)

def show_image(image, face, duration=500):
    '''Moves base motor by some relative angle (in radians)
    '''
    x,y,w,h = face
    cv2.ellipse(image, (x, y), (int(w/2), int(h/2)),
                 0, 0, 360, color=(0,0,255), thickness=4)
    cv2.imshow('', image)
    cv2.waitKey(duration)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = Camera()

    # if you want to visualize the face detection, uncomment this section
    # script will hand on this function until you close window
    draw_face = ShowFace(camera.unpack_configs(False))
    # camera.gui.show(window_name='Detecting Faces (press ESC to exit)',
                   # modifier_fns=[draw_face])


    # initialize the robot
    starting_arm_jpos = [0.0, -0.85, 1.0, 1.24, 0.0]
    robot = RobotArm()
    robot.move_arm_jpos(starting_arm_jpos)

    # Face Tracking code begins here
    delay = 0.5
    while True:
        time.sleep(delay)

        # get image from camera
        image = camera.get_image()

        # detect face
        face = find_face(image)
        if face is None:
            # no face found so go to next iteration
            continue

        # visualize the face, helpful for debugging
        show_image(image, face)

        # unpack info about face location and shape
        x,y,w,h = face

        #####################################
        # your code goes here
        #####################################
        cx, cy = image.shape/2
        move_base_relative(robot, -(x-cx)/600)

        #####################################
        #####################################


