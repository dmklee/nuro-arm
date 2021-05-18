##############################################################################
# Author: David Klee
# Date: May 18, 2021
#
#
##############################################################################
import numpy as np
import cv2

from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.robot.robot_arm import RobotArm
from ..utils import ShowFace, get_face

camera = Camera()

# if you want to visualize the face detection, uncomment this section
# script will hand on this function until you close window
camera.gui.show(window_name='Detecting Faces (press ESC to exit)',
               modifier_fns=[ShowFace(camera.unpack_configs(False))])


# initialize the robot
starting_arm_jpos = [0.0, -0.85, 1.0, 1.24, 0.0]
robot = RobotArm()
robot.move_arm_jpos(starting_arm_jpos)

# create function for relative movement of base motor
def move_base_relative(robot, rel_angle):
    '''Moves base motor by some relative angle (in radians)
    '''
    base_servo_idx = 0
    arm_jpos = robot.get_arm_jpos()

    # adjust the angle of the base servo
    arm_jpos[base_servo_idx] += rel_angle

    # move to adjusted postion
    robot.move_arm_jpos(arm_jpos)


# Face Tracking code begins here
delay = 0.5
while True:
    time.sleep(delay)

    # get image from camera
    image = camera.get_image()

    # detect face
    face = get_face(image)
    if face is None:
        # no face found so go to next iteration
        continue

    # unpack info about face location and shape
    x,y,w,h = face

    #####################################
    # your code goes here
    #####################################
    move_base_relative(robot, -x/300)

    #####################################
    #####################################


