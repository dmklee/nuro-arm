import numpy as np
import pybullet as pb


from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.constants import default_cam_pose_mtx
import time

if __name__ == "__main__":
    robot = RobotArm('sim')
    robot.close_gripper()

    robot.add_camera(default_cam_pose_mtx)

    pos = np.zeros(3)
    pos[0] = np.random.uniform(-0.2, 0.2)
    pos[1] = np.random.uniform(0.1, 0.2)
    pos[2] = np.random.uniform(0.013, 0.4)
    euler = (0,0,0)
    cube_id = robot.controller.add_cube(pos, euler)

    cube_pos, cube_quat = pb.getBasePositionAndOrientation(cube_id,
                                                           robot.controller._client
                                                          )
    cube_euler = pb.getEulerFromQuaternion(cube_quat)

    robot.open_gripper()
    time.sleep(1)
    robot.move_hand_to(cube_pos)
    print(cube_pos)
    print(robot.controller._get_link_pose('virtual_palm')[0])
    time.sleep(10)
    robot.close_gripper()
    time.sleep(1)

    while True:
        time.sleep(0.1)
