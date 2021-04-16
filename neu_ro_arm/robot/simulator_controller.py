import numpy as np
import pybullet as pb

from robot.base_controller import BaseController
from robot.motion_planner import PybulletBase

class SimulatorController(BaseController, PybulletBase):
    def __init__(self):
        PybulletBase.__init__(self, pb.GUI)
        pb.setRealTimeSimulation(1, self._client)

    def move_command(self, j_idxs, jpos):
        num_joints = len(j_idxs)
        pb.setJointMotorControlArray(self.robot_id,
                                     j_idxs,
                                     pb.POSITION_CONTROL,
                                     jpos,
                                    )

    def read_command(self, j_idxs):
        jpos = [pb.getJointState(self.robot_id, j_idx, physicsClientId=self._client)[0]
                           for j_idx in j_idxs]
        return jpos

