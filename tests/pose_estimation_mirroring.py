import numpy as np
import time
import pybullet as pb
import pybullet_data
import cv2 as cv

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.robot.motion_planner import MotionPlanner
from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.camera import camera_utils
from neu_ro_arm import constants

camera = Camera()
xarm = RobotArm()
xarm.mp = MotionPlanner(headless=False)
xarm._mirror_planner()
xarm.mp.add_camera(camera._cam2world)

# create cube
cube_obj = None

xarm.passive_mode()

t = time.time()
while 1:
    xarm._mirror_planner()
    try:
        rvecs = get_checkerboard(camera)
        cube = camera_utils.find_cubes(camera.get_image(),
                                        camera._mtx,
                                        camera._dist_coeffs,
                                        camera._cam2world)[0]
        if cube_obj is None:
            cube_obj = xarm.mp.add_cube(cube.pos, cube.euler)
            pb.changeVisualShape(cube_obj, -1, rgbaColor=[0,1,0,0.5],
                                 physicsClientId=xarm.mp._client)
        else:
            quat = pb.getQuaternionFromEuler(cube.euler)
            pb.resetBasePositionAndOrientation(cube_obj, cube.pos, quat,
                                               xarm.mp._client)
    except IndexError:
        pass
    time.sleep(0.1)

while 1:
    time.sleep(0.1)
