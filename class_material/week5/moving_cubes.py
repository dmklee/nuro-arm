import numpy as np
import time

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.camera.gui import GUI, ImageModifierFunction
from neu_ro_arm.camera import camera_utils

cam = Camera()
img = cam.get_image()

# tag_ids = [0,1,2]

# cam = Camera()
# modifier = ShowCubes(cam.unpack_configs(False), include_id=True)
# exit_keys = [ord(str(i)) for i in tag_ids]
# exit_key = cam.gui.show(modifier_fns=[modifier],
             # exit_keys=exit_keys)

# if exit_key == 27:
    # exit()

# tag_id = exit_keys.index(exit_key)
# img = cam.get_image()
# cubes = camera_utils.find_cubes(img, cam._mtx, cam._dist_coeffs, cam._cam2world)

# cube = next((c for c in cubes if c.id_==tag_id), None)
# if cube is None:
    # print('That cube is not visible')

# pos = cube.pos
# pos[2] = max(0.015, pos[2])

# pos[1] -= 0.015

# robot = RobotArm()
# robot.open_gripper()
# print(robot.move_hand_to(pos))

# robot.close_gripper()
# robot.home()
