import numpy as np
import matplotlib.pyplot as plt
import time

from nuro_arm.robot.robot_arm import RobotArm
from nuro_arm.robot.xarm_controller import itos

robot = RobotArm()
robot.passive_mode()
print('here')
time.sleep(0.5)

joint = 'base'
j_id = robot.controller.get_joint_id(joint)

angles = np.linspace(0, 2*np.pi, num=200)
vals = np.pi/2 * np.sin(angles)

while 1:
    for v in vals:
        robot.controller._move_servo(j_id, v, duration=20)
        # time.sleep(0.0)
