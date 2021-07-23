import pybullet as pb
import time
import numpy as np

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.constants import GRIPPER_CLOSED, GRIPPER_OPENED

PICK = np.array([-0.029,0.385,-1.051,1.931,0])
START = np.array([-0.029,0.038,-0.892,0.628,0])
GOAL = np.array([-0.029,-0.671,-0.237,0.303,0])

robot = RobotArm('real')

robot.open_gripper()
robot.move_arm_jpos(PICK)
robot.close_gripper()
robot.move_arm_jpos(START)
if input('Ready [y/N]') != 'y':
    exit()

robot.controller.write_arm_jpos(GOAL, speed=4.0)
time.sleep(0.08)
robot.controller.write_gripper_state(0.6, speed=4)
time.sleep(3)
