import numpy as np
import pybullet as pb
import time

from nuro_arm.robot.robot_arm import RobotArm
from nuro_arm.constants import CUBE_SIZE

robot = RobotArm('sim', headless=False)

# make GUI view better
pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                              cameraYaw=50,
                              cameraPitch=-40,
                              cameraTargetPosition=(-0.45, 0.35, -0.4))

robot.set_gripper_state(0.5)
client = robot.controller._client
pb.setGravity(0,0,0,client)

# create
id_ = pb.createVisualShape(pb.GEOM_BOX,
                           halfExtents=3*[CUBE_SIZE/2],
                           rgbaColor=[0.1,0.1,0.8,0.5])
pos_body = [0, 0, 0]
body = pb.createMultiBody(1, -1, id_, pos_body)

dbg_params = {
    'x' : pb.addUserDebugParameter('cube_x', rangeMin=0., rangeMax= 0.25,
                                startValue= 0.15, physicsClientId=client),
    'y' : pb.addUserDebugParameter('cube_y', rangeMin=-0.15, rangeMax= 0.15,
                                startValue= 0.0, physicsClientId=client),
    'z' : pb.addUserDebugParameter('cube_z', rangeMin=CUBE_SIZE/2, rangeMax= 0.45,
                                startValue= 0.2, physicsClientId=client),
}
d_toggle = pb.addUserDebugParameter('toggle orientation specification',
                                    1, 0, 0,
                                    physicsClientId=client)
dbg_params.update({
    'pitch': pb.addUserDebugParameter('gripper_pitch', rangeMin=0, rangeMax=np.pi,
                                    startValue= 2*np.pi, physicsClientId=client),
    'roll' : pb.addUserDebugParameter('gripper_roll', rangeMin=-np.pi/2, rangeMax= np.pi/2,
                                    startValue= 0.0, physicsClientId=client),
})

dbg_values = {d:0 for d,i in dbg_params.items()}
while True:
    button_val = pb.readUserDebugParameter(d_toggle, physicsClientId=client)
    reset_cube = False
    move_arm = False
    for name, prm in dbg_params.items():
        new_val = pb.readUserDebugParameter(prm, physicsClientId=client)
        if abs(new_val-dbg_values[name]) > 1e-4:
            dbg_values[name] = new_val
            if name in 'xyz':
                reset_cube = True
                move_arm = True
            elif name in ('pitch', 'roll') and button_val % 2 == 1:
                move_arm = True

    pos = (dbg_values['x'],dbg_values['y'],dbg_values['z'])
    if reset_cube:
        pb.resetBasePositionAndOrientation(body, pos, (0,0,0,1),physicsClientId=client)

    if move_arm:
        # first teleport to good initial arm jpos, this should be good for most
        # positions in the workspace
        if button_val % 2 == 0:
            robot.move_hand_to(pos)
        else:
            robot.move_hand_to(pos, (dbg_values['pitch'],dbg_values['roll']))
        print('done')
    time.sleep(0.1)
