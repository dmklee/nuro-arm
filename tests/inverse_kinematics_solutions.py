import pybullet as pb
import time

from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.constants import cube_size

PI = 3.141592653589793

robot = RobotArm('sim')
robot.set_gripper_state(0.5)
client = robot.controller._client
pb.setGravity(0,0,0,client)

# create
id_ = pb.createVisualShape(pb.GEOM_BOX,
                           halfExtents=3*[cube_size/2],
                           rgbaColor=[0.1,0.1,0.8,0.5])
pos_body = [0, 0, 0]
body = pb.createMultiBody(1, -1, id_, pos_body)

d_toggle = pb.addUserDebugParameter('toggle orientation specification',
                                    1, 0, 0,
                                    physicsClientId=client)
d_pitch = pb.addUserDebugParameter('gripper pitch', rangeMin=-PI, rangeMax=-PI/3,
                                    startValue= -2.5, physicsClientId=client)
d_roll = pb.addUserDebugParameter('gripper roll', rangeMin=-PI/2, rangeMax= PI/2,
                                    startValue= 0.0, physicsClientId=client)
d_x = pb.addUserDebugParameter('cube x', rangeMin=-0.15, rangeMax= 0.15,
                                startValue= 0.0, physicsClientId=client)
d_y = pb.addUserDebugParameter('cube y', rangeMin=0.05, rangeMax= 0.25,
                                startValue= 0.15, physicsClientId=client)
d_z = pb.addUserDebugParameter('cube z', rangeMin=cube_size/2, rangeMax= 0.3,
                                startValue= 0.1, physicsClientId=client)

def grab_specific_cube(robot, camera, tag_id):
    pass

while True:
    pitch = pb.readUserDebugParameter(d_pitch, physicsClientId=client)
    roll = pb.readUserDebugParameter(d_roll, physicsClientId=client)
    x = pb.readUserDebugParameter(d_x, physicsClientId=client)
    y = pb.readUserDebugParameter(d_y, physicsClientId=client)
    z = pb.readUserDebugParameter(d_z, physicsClientId=client)
    button_val = pb.readUserDebugParameter(d_toggle, physicsClientId=client)

    pb.resetBasePositionAndOrientation(body, (x,y,z), (0,0,0,1),physicsClientId=client)

    if button_val % 2 == 0:
        robot.move_hand_to((x,y,z), (pitch,roll))
    else:
        robot.move_hand_to((x,y,z))
    time.sleep(0.1)
