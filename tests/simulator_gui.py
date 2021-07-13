import pybullet as pb
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.constants import cube_size

PI = 3.141592653589793

robot = RobotArm('sim')
robot.set_gripper_state(0.5)
client = robot.controller._client

# create
dbg_params = {}
for name in robot.joint_names:
    start = 0
    if name == 'wrist': start = 1.1
    if name == 'wristRotation': start = 1.1
    if name == 'base': start = 1.1
    dbg_params[name] = pb.addUserDebugParameter(name, rangeMin=-2, rangeMax=2,
                                    startValue=start, physicsClientId=client)
dbg_params['gripper'] = pb.addUserDebugParameter('gripper', rangeMin=0, rangeMax=1,
                                startValue=0.5, physicsClientId=client)

dbg_values = {d:pb.readUserDebugParameter(i, physicsClientId=client) for d,i in dbg_params.items()}
while True:
    for name, prm in dbg_params.items():
        new_val = pb.readUserDebugParameter(prm, physicsClientId=client)
        if 1 or abs(new_val-dbg_values[name]) > 1e-4:
            dbg_values[name] = new_val
            if name == 'gripper':
                gripper_state = 0.05*(new_val) + 1.45*(1-new_val)
                pb.setJointMotorControlArray(robot.controller.robot_id,
                                             robot.controller.gripper_joint_idxs,
                                             pb.POSITION_CONTROL,
                                             2*[gripper_state],
                                             positionGains=2*[0.05],
                                             physicsClientId=client)
            else:
                pb.setJointMotorControl2(robot.controller.robot_id,
                                         robot.joint_names.index(name)+1,
                                         pb.POSITION_CONTROL,
                                         new_val,
                                         positionGain=0.1,
                                         physicsClientId=client)
    robot._mirror_planner()
    # [robot.controller.timestep() for _ in range(2)]

    time.sleep(0.1)
