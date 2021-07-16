import pybullet as pb
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.constants import cube_size

PI = 3.141592653589793

robot = RobotArm('sim', headless=False)
robot.set_gripper_state(0.5)
client = robot.controller._client

# create
dbg_params = {}
dbg_params['gripper'] = pb.addUserDebugParameter('gripper', rangeMin=0, rangeMax=1,
                                startValue=0.5, physicsClientId=client)
for name in reversed(robot.joint_names[:-1]):
    start = 0
    dbg_params[name] = pb.addUserDebugParameter(name, rangeMin=-2, rangeMax=2,
                                    startValue=start, physicsClientId=client)

dbg_values = {d:pb.readUserDebugParameter(i, physicsClientId=client)
              for d,i in dbg_params.items()}
while True:
    for name, prm in dbg_params.items():
        new_val = pb.readUserDebugParameter(prm, physicsClientId=client)
        if 1 or abs(new_val-dbg_values[name]) > 1e-4:
            dbg_values[name] = new_val
            if name == 'gripper':
                robot.controller.write_gripper_state(new_val)
            else:
                joint_id = robot.controller.get_joint_id(name)
                robot.controller._write_jpos([joint_id], [new_val])
    robot._mirror_planner()
    # [robot.controller.timestep() for _ in range(2)]

    time.sleep(0.1)
