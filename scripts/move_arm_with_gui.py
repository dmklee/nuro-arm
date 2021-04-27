import numpy as np
import sys
from neu_ro_arm.robot.robot_arm import RobotArm

def move_with_gui(robot_mode):
    '''Use GUI to control robot joints

    Currently, there is no collision detection during gui motions so the user
    should be careful and ensure no objects are within the reach of the arm

    Parameters
    ----------
    robot_mode : {'real', 'sim'}
    '''
    robot = RobotArm(robot_mode)
    robot.move_with_gui()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_mode = str(sys.argv[1])
    else:
        robot_mode = 'real'

    assert robot_mode in ('real','sim'), 'Invalid robot mode argument'

    robot = RobotArm(robot_mode)
    robot.move_with_gui()

