import time

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.camera import camera_utils
from neu_ro_arm.camera.gui import GUI, ShowCubes
from neu_ro_arm.constants import cube_size

def filter_cubes(cubes, cube_id):
    '''Returns cube with specific id if it is in list otherwise it returns None
    '''
    cube = next((c for c in cubes if c.id_==cube_id), None)
    return cube

def get_cubes(camera):
    '''Returns list of cubes that camera can see
    '''
    image = camera.get_image()
    cubes = camera_utils.find_cubes(image, camera._mtx,
                                    camera._dist_coeffs, camera._cam2world)
    return cubes

def get_user_choice(camera, title=''):
    '''Creates window with camera feed to allow user to specify cube id
    '''
    # create window that shows camera feed with cubes highlighted
    modifier = ShowCubes(camera.unpack_configs(False), include_id=True)
    exit_keys = [ord(str(i)[0]) for i in range(12)]
    exit_key = camera.gui.show(modifier_fns=[modifier],
                            exit_keys=exit_keys,
                            window_name=title)

    if exit_key == 27:
        # escape key was hit so exit program
        exit()

    cube_id = exit_keys.index(exit_key)
    return cube_id

def add_correction(position, correction):
    '''Adds each element in correction list to corresponding element
    in position list
    '''
    position[0] += correction[0] # x 
    position[1] += correction[1] # y 
    position[2] += correction[2] # z 
    return position

def pick(robot, position):
    '''Move robot gripper to specified position and close gripper.
    '''
    robot.open_gripper()
    time.sleep(0.5)
    robot.move_hand_to(position)
    time.sleep(0.5)
    robot.close_gripper()

def place(robot, position):
    '''Move robot gripper to specified position and open gripper.
    '''
    robot.move_hand_to(position)
    time.sleep(0.5)
    robot.open_gripper()

if __name__ == "__main__":
    POSITION_CORRECTION = [0.005, -0.009, 0.02] # units are in meters

    DEFAULT_ARM_JPOS = [0, -0.87, 1.3, 0.75, 0]

    # connect to robot
    robot = RobotArm()

    # connect to camera
    camera = Camera()

    #####################
    # pick up an object
    #####################
    cube_id = get_user_choice(camera, title='Choose cube id to pick up')
    cubes = get_cubes(camera)
    cube = filter_cubes(cubes, cube_id)
    if cube is None:
        print('cube was not found')
        exit()

    position = add_correction(cube.pos, POSITION_CORRECTION)

    pick(robot, position)
    #####################

    robot.move_arm_jpos(DEFAULT_ARM_JPOS)

    #####################
    # place an object
    #####################
    cube_id = get_user_choice(camera, title='Choose cube id to place upon')
    cubes = get_cubes(camera)
    cube = filter_cubes(cubes, cube_id)
    if cube is None:
        print('cube was not found')
        exit()

    position = add_correction(cube.pos, POSITION_CORRECTION)

    # we want to place ABOVE the cube so increase z-dimension
    position[2] += 1.2 * cube_size

    place(robot, position)
    #####################

    robot.move_arm_jpos(DEFAULT_ARM_JPOS)
