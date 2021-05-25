import numpy as np
import time

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.camera.gui import GUI, ShowCubes
from neu_ro_arm.camera import camera_utils

# I just picked a convenient spot to place arm
START_ARM_JPOS = [-0.15498, -1.0932, 1.386, 1.3362, 0.01256]

def decide_cube_id(camera):
    # setup visualizer so you can see camera image with identified cubes
    # when the user enters a number on the keypad, that will be returned
    modifier = ShowCubes(camera.unpack_configs(False), include_id=True)
    exit_keys = [ord(str(i)[0]) for i in range(12)]
    exit_key = camera.gui.show(modifier_fns=[modifier],
                            exit_keys=exit_keys,
                            window_name='Press cube number to grab it',
                           )

    # escape key was pressed so stop program
    if exit_key == 27:
        exit()

    cube_id = exit_keys.index(exit_key)
    return cube_id

def pickup_cube(camera, robot, cube_id):
    # identify all cubes in current image
    img = camera.get_image()
    cubes = camera_utils.find_cubes(img, camera._mtx, camera._dist_coeffs, camera._cam2world)

    # filter the list of cubes for the one with the right cube_id
    cube = next((c for c in cubes if c.id_==cube_id), None)
    if cube is None:
        print(f'Could not find the cube={cube_id}')
        return

    # get cube location
    cube_pos = cube.pos

    # perform correction
    cube_pos[0] += POSITION_CORRECTION[0] # x 
    cube_pos[1] += POSITION_CORRECTION[1] # y 
    cube_pos[2] += POSITION_CORRECTION[2] # z 

    # go grab it with robot
    robot.open_gripper()
    robot.move_hand_to(cube_pos)
    time.sleep(0.2)
    robot.close_gripper()

    # return to start position
    robot.move_arm_jpos(START_ARM_JPOS)

    # drop block
    robot.open_gripper()

if __name__ == "__main__":
    # you will have to adjust this for your own robot
    POSITION_CORRECTION = [0, -0.018, 0.025]

    camera = Camera()
    robot = RobotArm()
    robot.move_arm_jpos(START_ARM_JPOS)

    cube_id = decide_cube_id(camera)

    pickup_cube(camera, robot, cube_id)
