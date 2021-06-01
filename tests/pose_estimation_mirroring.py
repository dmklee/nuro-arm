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

# xarm.move_arm_jpos([0, -1.0932, 1.386, 1.3362, 0.01256])
xarm.passive_mode()

other_cube = xarm.mp.add_cube([0,0,0],[0,0,0])
t = time.time()
while 1:
    pb.resetBasePositionAndOrientation(other_cube, pos, [0,0,0,1],
                                       physicsClientId=xarm.mp._client)
    # xarm._mirror_planner()
    # try:
        # rvecs = get_checkerboard(camera)
        # cube = camera_utils.find_cubes(camera.get_image(),
                                        # camera._mtx,
                                        # camera._dist_coeffs,
                                        # camera._cam2world)[0]
        # if cube_obj is None:
            # cube_obj = xarm.mp.add_cube(cube.pos, cube.euler)
            # pb.changeVisualShape(cube_obj, -1, rgbaColor=[0,1,0,0.5],
                                 # physicsClientId=xarm.mp._client)
        # else:
            # quat = pb.getQuaternionFromEuler(cube.euler)
            # pb.resetBasePositionAndOrientation(cube_obj, cube.pos, quat,
                                               # xarm.mp._client)
    # except IndexError:
        # pass
    time.sleep(0.1)
    # if (time.time()-t) > 4:
        # xarm.open_gripper()
        # xarm.move_hand_to(cube.pos, n_repeats=5, n_iters=100)
        # break

while 1:
    time.sleep(0.1)

# from PIL import Image
# A = np.zeros((10,8,3), dtype=np.uint8)
# A[::2,1::2,:] = 255
# im = Image.fromarray(A)
# im.save("neu_ro_arm/assets/checkerboard.png")

# exit()

xarm = RobotArm('sim')
client = xarm.controller._client
pb.setGravity(0,0,0,client)
# camera = Camera()
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# create piece of paper
obj_id = pb.loadURDF('neu_ro_arm/assets/urdf/checkerboard.urdf',
                     basePosition=[0,0.5,0.1],
                     physicsClientId=client)
tex_id = pb.loadTexture('neu_ro_arm/assets/checkerboard.png')
pb.changeVisualShape(obj_id, -1, textureUniqueId=tex_id, physicsClientId=client)
while 1:
    time.sleep(0.5)
