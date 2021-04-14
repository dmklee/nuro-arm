import numpy as np
import pybullet as pb

from src.base_controller import BaseController

class SimulatorController(BaseController):
    def __init__(self, robot_id):
        self.id = robot_id
        self._gripper_closed = [-0.3, -0.3]
        self._gripper_opened = [0.3, 0.3]
        self._arm_joint_idxs = [1,2,3,4,5]
        self._gripper_joint_idxs = [6,7]

    def move_command(self, j_idxs, jpos):
        num_joints = len(j_idxs)
        pb.setJointMotorControlArray(self.id,
                                     j_idxs,
                                     pb.POSITION_CONTROL,
                                     jpos,
                                    )

    def read_command(self, j_idxs):
        jpos = [pb.getJointInfo(self.id, j_idx)[1]
                           for j_idx in j_idxs]
        return jpos

