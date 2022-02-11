import numpy as np
import time

from nuro_arm.robot.robot_arm import RobotArm
from nuro_arm.camera.camera import Camera
from nuro_arm.cube import Cube
from nuro_arm.camera import camera_utils

xarm = RobotArm(headless=False)
xarm.mirror_planner()

camera = Camera(free_floating=False)

# create cube
cube = Cube(pos=(0,0,100), tag_id=0)

xarm.passive_mode()

t = time.time()
while 1:
    xarm.mirror_planner()
    try:
        cube_info = camera_utils.find_cubes(camera.get_image(), camera._cam2world,)[0]
        cube.reset_pose(cube_info.pos, cube_info.quat)
    except IndexError:
        pass
    time.sleep(0.1)
