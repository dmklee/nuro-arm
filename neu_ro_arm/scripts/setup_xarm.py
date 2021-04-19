import numpy as np
from robot.xarm_controller import XArmController
from robot.robot_arm import RobotArm

def main():
    # this tests the connection
    xarm = XArmController()

    # this calibrates the arm + gripper
    print('Calibration of the xarm will begin shortly.')
    print('Ensure the camera is not attached to the robot base')
    success, data = xarm.calibrate_arm()
    # success, data = xarm.calibrate()

    if success:
        np.savez(xarm.CONFIG_FILE, **data)
    else:
        print('Calibration failed. Please try again.')

def main2():
    robot = RobotArm('real')
    # robot.move_with_gui()

if __name__ == "__main__":
    main()
    import time
    time.sleep(10)

