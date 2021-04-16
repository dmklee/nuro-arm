import numpy as np
from robot.xarm_controller import XArmController

def main():
    # this tests the connection
    xarm = XArmController('real')

    # this calibrates the arm + gripper
    print('Calibration of the xarm will begin shortly.')
    print('Ensure the camera is not attached to the robot base')
    success, data = robot.controller.calibrate()

    if success:
        np.savez(xarm.CONFIG_FILE, **data)
    else:
        print('Calibration failed. Please try again.')

if __name__ == "__main__":
    main()

