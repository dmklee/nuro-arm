import numpy as np
from neu_ro_arm.robot.xarm_controller import XArmController

def calibrate():
    '''Calibrates servos in xArm so that the internal motion planner is accurate

    Calibration info is saved to a config file so this only needs to be performed
    once after the robot is assembled. For more information, see the
    installation guide
    '''
    # this tests the connection
    xarm = XArmController()

    # reset arm motor direction corrections
    xarm.arm_motor_directions = {k:1 for k in xarm.arm_motor_directions.keys()}

    # this calibrates the arm + gripper
    print('Calibration of the xarm will begin shortly.')
    print('Ensure the camera is not attached to the robot base')
    success, data = xarm.calibrate()

    if success:
        np.savez(xarm.CONFIG_FILE, **data)
    else:
        print('Calibration failed. Please try again.')

    return success

if __name__ == "__main__":
    calibrate()

