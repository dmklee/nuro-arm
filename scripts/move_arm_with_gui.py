import numpy as np
import sys
from neu_ro_arm.robot.robot_arm import RobotArm

def move_with_gui(robot_mode):
    '''Use GUI to control robot joints

    Currently, there is no collision detection during gui motions so the user
    should be careful and ensure no objects are within the reach of the arm

    In simulator, a cube will be added just for fun

    Parameters
    ----------
    robot_mode : {'real', 'sim'}
    '''
    robot = RobotArm(robot_mode)
    if robot_mode == 'sim':
        robot.controller.add_cube(np.array((0.01, 0.2, 0.015)),
                                  np.zeros(3) )
    robot.move_with_gui()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_mode = str(sys.argv[1])
    else:
        robot_mode = 'real'

    assert robot_mode in ('real','sim'), 'Invalid robot mode argument'

    move_with_gui(robot_mode)

