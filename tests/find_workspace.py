# the point of this is to find the best orientation of the xarm in terms of
# the workspace volume
import pybullet as pb
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

from nuro_arm.robot.robot_arm import RobotArm

N_GRIPPER_THETAS = 5
gripper_thetas = np.linspace(-np.pi/2, np.pi/2, num=N_GRIPPER_THETAS)

XY_SPACING = 0.01
WIDTH = 0.08
HEIGHT = 0.08
X_CENTER = 0.16
Y_CENTER = 0.0
x_points = np.arange(X_CENTER-HEIGHT, X_CENTER+HEIGHT+XY_SPACING, XY_SPACING)
y_points = np.arange(Y_CENTER-WIDTH, Y_CENTER+WIDTH+XY_SPACING, XY_SPACING)
N_BASE_HEIGHTS = 5
BASE_HEIGHT_SPACING = 0.02
base_heights = BASE_HEIGHT_SPACING*(np.arange(N_BASE_HEIGHTS)-N_BASE_HEIGHTS//2)
xyz_points = np.stack(np.meshgrid(x_points, y_points, base_heights)).T.reshape(-1,3)
print(xyz_points.shape)

feasiblility = np.zeros(xyz_points.shape[:-1], dtype=int)
print(feasiblility.shape)

robot = RobotArm('sim', headless=False, realtime=False)
robot.close_gripper()
pb.removeBody(robot._sim.plane_id)

id_ = pb.createVisualShape(pb.GEOM_BOX,
                           halfExtents=[0.005, 0.005, 0.01],
                           rgbaColor=[0.1,0.8,0.2,0.5])
pos_body = [0, 0, 0]
body = pb.createMultiBody(1, -1, id_, pos_body)

for pos in xyz_points:
    pb.resetBasePositionAndOrientation(body, pos, (0,0,0,1))
    for roll in gripper_thetas:
        yaw = np.arctan2(pos[1], pos[0])
        rot = R.from_euler('z', yaw) * R.from_euler('YZ', (np.pi, roll+np.pi))
        rot = rot.as_quat()

        jpos, info = robot.mp.calculate_ik(pos, rot)
        collisions = robot.mp.find_collisions(jpos, ignore_gripper=1)
        if len(collisions) > 0:
            break

        achieved_pos, achieved_rot = robot._sim.get_hand_pose()
        time.sleep(0.01)

# while 1:
    # pass
